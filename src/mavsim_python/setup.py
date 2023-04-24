"""setup.py install python code

Copyright (c) (2021) Greg Droge
"""
import os

from setuptools import setup

thelibFolder = os.path.dirname(os.path.realpath(__file__))
requirementPath = thelibFolder + "/requirements.txt"

install_requires = []
if os.path.isfile(requirementPath):
    with open(requirementPath) as f:
        install_requires = f.read().splitlines()

setup(name="mav_sim", install_requires=install_requires, packages=["mav_sim"])
