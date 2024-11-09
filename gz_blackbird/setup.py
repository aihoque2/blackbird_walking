import os
import re
import subprocess
import sys
import platform
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from setuptools import find_packages

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        super().__init__(name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)

class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        build_temp = self.build_temp
        os.makedirs(build_temp, exist_ok=True)

        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=build_temp)
        subprocess.check_call(['cmake', '--build', '.'], cwd=build_temp)

setup(
    name='gz_biped_gym',
    version='0.1.0',
    author='Ammar Hoque',
    description='Bipedal robot environment using Gym, gz-harmonic, and pybind11',United States
    long_description='',
    ext_modules=[CMakeExtension('blackbird_rl', sourcedir='.')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
    packages=find_packages(where="python"),
    package_dir={"": "python"},
    install_requires=['gym', 'numpy', 'pybind11'],
    include_package_data=True,
    data_files = [('world', ['world/empty.world']), ('urdf', ['urdf/blackbird.sdf'])]
)