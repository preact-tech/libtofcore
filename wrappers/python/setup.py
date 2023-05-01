import os
import sys
import subprocess
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

# References: https://github.com/pybind/cmake_example 

# A CMakeExtension needs a sourcedir instead of a file list.
# The name must be the _single_ output extension from the CMake build.
# If you need multiple extensions, see scikit-build.
class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = "") -> None:
        super().__init__(name, sources=[])
        self.sourcedir = os.fspath(Path(sourcedir).resolve())


class CMakeBuild(build_ext):
    def build_extension(self, ext: CMakeExtension) -> None:

        # Must be in this form due to bug in .resolve() only fixed in Python 3.10+
        ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)
        extdir = ext_fullpath.parent.resolve()

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
        ]
        build_args = []

        # gets passed to CmakeList.txt
        cmake_args += [f"-DVERSION_PACKAGE={self.distribution.get_version()}"]

        build_temp = Path(self.build_temp) / ext.name
        if not build_temp.exists():
            build_temp.mkdir(parents=True)

        subprocess.run(
            ["cmake", ext.sourcedir] + cmake_args, cwd=build_temp, check=True
        )
        subprocess.run(
            ["cmake", "--build", "."] + build_args, cwd=build_temp, check=True
        )

setup(
    name="pytofcore",
    version="0.0.1",
    author="Miguel Gonzalez",
    author_email="miguel.gonzalez@preact-tech.com",
    description="Library that interfaces with PreAct ToF devices",
    ext_modules=[CMakeExtension("libtofcore")],
    cmdclass={"build_ext": CMakeBuild},
    python_requires=">=3.8",
    extras_require={"test": ["pytest>=6.0"]},
    zip_safe=False
)