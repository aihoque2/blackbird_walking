import os

for old_name in os.listdir():
    if old_name != 'add_underscore.py':
        print("old_name: ", old_name)
        name = old_name[0:len(old_name)-4]
        name = name.replace(" ", "_")
        name = name.replace(".", "_")
        name += '.stl'
        print("name: ", name)
        print()
        os.rename(old_name, name)
