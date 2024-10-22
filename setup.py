#!/usr/bin/env python

import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'system_monitor'

setup(name=package_name,
      version='0.0.0',
      packages=find_packages(),
      data_files=[
          ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
          ('share/' + package_name, ['package.xml']),
          (f'share/{package_name}/launch', glob.glob('./launch/*.launch.py')),
          (f'share/{package_name}/config', glob.glob('./config/*.yaml')),
      ],
      maintainer='Ralf Kaestner',
      maintainer_email='ralf.kaestner@gmail.com',
      description='System monitoring tools for ROS',
      license='GNU Lesser General Public License (LGPL)',
      tests_require=['pytest'],
      entry_points={
          'console_scripts': [
              f'cpu_monitor = {package_name}.cpu_monitor:main',
              f'hdd_monitor = {package_name}.hdd_monitor:main',
              f'mem_monitor = {package_name}.mem_monitor:main',
              f'net_monitor = {package_name}.net_monitor:main',
              f'ntp_monitor = {package_name}.ntp_monitor:main',
          ],
      })
