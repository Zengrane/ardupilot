from setuptools import setup, Extension
from Cython.Build import cythonize
from Cython.Distutils import build_ext

cythonize("dubins/dubins.pyx")

ext_modules = [
    Extension("dubins",
        ["dubins/src/dubins.c", "dubins/dubins.pyx"],
        include_dirs = ["dubins/include"],
    )
]

setup(
    name="dubins",
    ext_modules = ext_modules,
    cmdclass     = {'build_ext' : build_ext}
)
