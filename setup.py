#!/usr/bin/env python

# Copyright (c) 2024 SoftBank Corp.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

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
