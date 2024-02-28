"""
This file follows a similar setup that Intel RealSense uses for their SDK
https://github.com/IntelRealSense/librealsense
"""

import os
from setuptools import setup, find_packages


package_name = "pytofcore"
package_data = {}

if os.name == 'posix':
    package_data[package_name] = ['*.so']
else:
    package_data[package_name] = ['*.pyd', '*.dll']


# This creates a list which is empty but returns a length of 1.
# Should make the wheel a binary distribution and platlib compliant.
class EmptyListWithLength(list):
    def __len__(self):
        return 1


# The information here can also be placed in setup.cfg - better separation of
# logic and declaration, and simpler if you include description/version in a file.
setup(
    name=package_name,
    version="1.2.0",
    author="Miguel Gonzalez",
    author_email="miguel.gonzalez@preact-tech.com",
    description="Module for interfacing with with PreAct ToF devices",
    long_description="",
    ext_modules=EmptyListWithLength(),
    package_data=package_data,
    include_package_data=True,
    packages=find_packages(),
    zip_safe=False,
    extras_require={"test": ["pytest>=6.0"]},
    python_requires=">=3.7",
)