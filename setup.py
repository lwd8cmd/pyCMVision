#!/usr/bin/python
#
# python-pyXiQ
#
# 2015 Lauri Hamarik
#
# I, the copyright holder of this file, hereby release it into the
# public domain. This applies worldwide. In case this is not legally
# possible: I grant anyone the right to use this work for any
# purpose, without any conditions, unless such conditions are
# required by law.
#
# Usage: sudo python setup.py install

from distutils.core import Extension, setup
setup(
    name = "pyCMVision",
    version = "0.9",
    author = "Lauri Hamarik",
    description = "Computer vision lib for v4l2 cameras",
    license = "Public Domain",
    classifiers = [
        "License :: Public Domain",
        "Programming Language :: C"],
    ext_modules = [
        Extension("pyCMVision", ["pyCMVision.c"], libraries = ["v4l2"])])
