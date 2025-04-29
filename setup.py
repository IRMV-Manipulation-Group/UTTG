#   ************************************************************************\
#
#                                C O P Y R I G H T
#
#     Copyright © 2024 IRMV lab, Shanghai Jiao Tong University, China.
#                           All Rights Reserved.
#
#     Licensed under the Creative Commons Attribution-NonCommercial 4.0
#     International License (CC BY-NC 4.0).
#     You are free to use, copy, modify, and distribute this software and its
#     documentation for educational, research, and other non-commercial purposes,
#     provided that appropriate credit is given to the original author(s) and
#     copyright holder(s).
#
#     For commercial use or licensing inquiries, please contact:
#     IRMV lab, Shanghai Jiao Tong University at: https://irmv.sjtu.edu.cn/
#
#                                D I S C L A I M E R
#
#     IN NO EVENT SHALL TRINITY COLLEGE DUBLIN BE LIABLE TO ANY PARTY FOR
#     DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING,
#     BUT NOT LIMITED TO, LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE
#     AND ITS DOCUMENTATION, EVEN IF TRINITY COLLEGE DUBLIN HAS BEEN ADVISED OF
#     THE POSSIBILITY OF SUCH DAMAGES.
#
#     TRINITY COLLEGE DUBLIN DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED
#     TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#     PURPOSE. THE SOFTWARE PROVIDED HEREIN IS ON AN "AS IS" BASIS, AND TRINITY
#     COLLEGE DUBLIN HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
#     ENHANCEMENTS, OR MODIFICATIONS.
#
#     The authors may be contacted at the following e-mail addresses:
#
#             YX.E.Z yixuanzhou@sjtu.edu.cn
#
#     Further information about the IRMV and its projects can be found at the ISG web site :
#
#            https://irmv.sjtu.edu.cn/
#
#   \*************************************************************************
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import sys
import os
import subprocess

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        super().__init__(name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)

class CMakeBuild(build_ext):
    def run(self):
        try:
            subprocess.check_call(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = [
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
            '-DPYTHON_EXECUTABLE=' + self.get_python_executable(),
            '-DCOMPILE_UTTG_PYBINDING=ON'
        ]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=self.build_temp)

    def get_python_executable(self):
        return os.path.abspath(sys.executable)


setup(
    name='UTTG_interface_py',
    version='1.2.1',
    author='YX.E.Z',
    author_email='yixuanzhou@sjtu.edu.cn',
    description='A package for Arm Servo Mode Interface',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://irmv.sjtu.edu.cn/',
    packages=['interface'],
    ext_modules=[CMakeExtension('UTTG_interface_py', sourcedir='.')],
    cmdclass={'build_ext': CMakeBuild},
    zip_safe=False,
    install_requires=[  ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: Creative Commons Attribution-NonCommercial 4.0 International License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.8',
)