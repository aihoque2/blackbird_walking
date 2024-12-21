#include "PosePublisher.h"

namespace blackbird_ros2{

    BlackbirdPosePublisher::BlackbirdPosePublisher(){
        node_ = rclcpp::Node::make_shared("pose_publisher");
        pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("gz_state", 10);
        x = 0.0;
        y = 0.0;
        z = 0.0;
        r = 0.0;
        p = 0.0;
        w = 0.0;

    }


    BlackbirdPosePublisher::~BlackbirdPosePublisher(){
        rclcpp::shutdown();
    }

    void BlackbirdPosePublisher::Configure(const gz::sim::Entity&,
                        const std::shared_ptr<const sdf::Element>&,
                        gz::sim::EntityComponentManager& ecm,
                        gz::sim::EventManager& )
    {
        // TODO: Initialize velocity to zero
        gz::sim::Entity torso_ent = ecm.EntityByComponents(gz::sim::components::Name("torso"), gz::sim::components::Link());

        if (torso_ent == gz::sim::kNullEntity)
        {
            throw std::runtime_error("StateUpdater::Configure() Torso link entity not found.");
        } 

        gz::sim::Link torso_link(torso_ent);
        auto world_pose_opt = torso_link.WorldPose(ecm);
        if (!world_pose_opt)
        {
            throw std::runtime_error("StateUpdater::Configure() Failed to get WorldPose for the torso link.");
        }
        const gz::math::Pose3d &initial_pose = *world_pose_opt;

        x = initial_pose.Pos().X();
        y = initial_pose.Pos().Y();
        z = initial_pose.Pos().Z();
        r = initial_pose.Rot().Roll();
        p = initial_pose.Rot().Pitch();
        w = initial_pose.Rot().Yaw();

        // intregrate contacts
        for (auto link_name: LINK_NAMES){
            auto link_ent = ecm.EntityByComponents(gz::sim::components::Name(link_name), gz::sim::components::Link());
            std::vector<gz::sim::Entity> collisions = ecm.ChildrenByComponents(link_ent, gz::sim::components::Collision());
            
            if (collisions.empty())
            {
                throw std::runtime_error("BlackbirdPosePublisher::Configure() parent_ent has no collisions...wtf");
            }

            for (auto collision: collisions)
            {
                auto collision_name_opt = ecm.ComponentData<gz::sim::components::Name>(collision);
                std::string collision_name = collision_name_opt.value();

                auto contact = ecm.Component<gz::sim::components::ContactSensorData>(collision);
                if (contact == nullptr)
                {
                    std::cout << "ContactPlugin::Configure() creating contact component for: " << link_name  << std::endl;
                    ecm.CreateComponent(collision, gz::sim::components::ContactSensorData());
                }

                link_map[link_name].push_back(collision_name); // ez debugging this method
            }
        }


    }

   void BlackbirdPosePublisher::PostUpdate(const gz::sim::UpdateInfo &info,
                                    const gz::sim::EntityComponentManager &ecm)
    {
        // TODO: update velocities before establishing new poses
        // TODO: Initialize velocity to zero
        gz::sim::Entity torso_ent = ecm.EntityByComponents(gz::sim::components::Name("torso"), gz::sim::components::Link());

        if (torso_ent == gz::sim::kNullEntity)
        {
            throw std::runtime_error("StateUpdater::Configure() Torso link entity not found.");
        } 

        gz::sim::Link torso_link(torso_ent);
        auto torso_pose_opt = torso_link.WorldPose(ecm);
        if (!torso_pose_opt)
        {
            throw std::runtime_error("StateUpdater::Configure() Failed to get WorldPose for the torso link.");
        }
        const gz::math::Pose3d &torso_pose = *torso_pose_opt;

        double x_new = torso_pose.Pos().X();
        double y_new = torso_pose.Pos().Y();
        double z_new = torso_pose.Pos().Z();
        double r_new = torso_pose.Rot().Roll();
        double p_new = torso_pose.Rot().Pitch();
        double w_new = torso_pose.Rot().Yaw();
        std::vector<double> state_vec(35, 0.0);
        state_vec[0] = x_new;
        state_vec[1] = y_new;
        state_vec[2] = z_new;
        state_vec[3] = r_new;
        state_vec[4] = p_new;
        state_vec[5] = w_new;

        // velocity calculations
        double dub_dt = std::chrono::duration<double>(info.dt).count();
        
        double x_vel = (x_new - x)/dub_dt;
        x = x_new;
        state_vec[6] = x_vel;

        double y_vel = (y_new - y)/dub_dt;
        y = y_new;
        state_vec[7] = y_vel;

        double z_vel = (z_new - z)/dub_dt;
        z = z_new;
        state_vec[8] = z_vel;
        
        double r_vel = (r_new - r)/dub_dt;
        r = r_new;
        state_vec[9] = r_vel;

        double p_vel = p_new - p/dub_dt;
        p = p_new;
        state_vec[10] = p_vel;

        double w_vel = (w_new - w)/dub_dt;
        w = w_new;
        state_vec[11] = w_vel;

        // joints
        int i = 12;
        for (auto& joint_name : JOINT_NAMES){
            auto joint_ent = ecm.EntityByComponents(gz::sim::components::Name(joint_name), gz::sim::components::Joint());
            
            auto* j_pos = ecm.Component<gz::sim::components::JointPosition>(joint_ent);
            if (j_pos == nullptr){
                throw std::runtime_error("Joint " + joint_name + " has no component JointPosition");
            }
            auto* j_vel = ecm.Component<gz::sim::components::JointVelocity>(joint_ent);
            if (j_vel == nullptr){
                throw std::runtime_error("Joint " + joint_name + " has no component JointVelocity");
            }
            state_vec[i] = j_pos->Data()[0];
            state_vec[i+10] = j_vel->Data()[0];
            i++;            
        }

        // contact sensor info
        i = 32; 
        for (auto& link_name : LINK_NAMES){
            bool contacted = 0;
            gz::sim::Entity link_ent = ecm.EntityByComponents(gz::sim::components::Name(link_name), gz::sim::components::Link());
            std::vector<gz::sim::Entity> collision_ents = ecm.ChildrenByComponents(link_ent, gz::sim::components::Collision());
            std::vector<std::string>& collision_names = link_map[link_name];

            for (auto& collision: collision_ents){
                auto contact = ecm.Component<gz::sim::components::ContactSensorData>(collision);
                if (contact == nullptr){
                    throw std::runtime_error("BlackbirdPosePublisher::PostUpdate() contact not found for collision");
                }

                if (contact->Data().contact_size() > 0){
                    contacted |= 1;
                }
                else{
                    contacted |= 0;
                }
            }
            // std::cout << "here's contacted for link " << link_name << ": " << contacted << std::endl;
            state_vec[i] = contacted;
            i++;
        }
        if (i!=35){
            throw std::runtime_error("BlackbirdPosePublisher::PostUpdate() i != 35");

        }
        msg_.data = std::move(state_vec);

        pub_->publish(msg_);

    }
}
GZ_ADD_PLUGIN(
    blackbird_ros2::BlackbirdPosePublisher,
    gz::sim::System,
    blackbird_ros2::BlackbirdPosePublisher::ISystemConfigure,
    blackbird_ros2::BlackbirdPosePublisher::ISystemPostUpdate
    
)