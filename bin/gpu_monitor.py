#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# \author Kevin Watts
# \brief Publishes diagnostic data on temperature and usage for a Quadro 600 GPU

import math
import subprocess

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Float32

from system_monitor.msg import GPUStatus

MAX_FAN_RPM = 4500


class GPUMonitor(object):
    def __init__(self):
        self._pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=10)
        self._usage_pub = rospy.Publisher('~usage', Float32, queue_size=10)
        self._mem_usage_pub = rospy.Publisher(
            '~mem_usage', Float32, queue_size=10)

    def pub_status(self):
        gpu_stat = GPUStatus()
        stat = DiagnosticStatus()
        try:
            card_out = self.get_gpu_status()
            gpu_stat = self.parse_smi_output(card_out)
            stat = self.gpu_status_to_diag(gpu_stat)
        except Exception:
            import traceback
            rospy.logerr('Unable to process nVidia GPU data')
            rospy.logerr(traceback.format_exc())

        array = DiagnosticArray()
        array.header.stamp = rospy.get_rostime()

        array.status = [stat]

        self._pub.publish(array)
        # self._gpu_pub.publish(gpu_stat)

    def _rads_to_rpm(self, rads):
        return rads / (2 * math.pi) * 60

    def _rpm_to_rads(self, rpm):
        return rpm * (2 * math.pi) / 60

    def get_gpu_status(self):
        p = subprocess.Popen('nvidia-smi -a', stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, shell=True)
        (o, e) = p.communicate()

        if not p.returncode == 0:
            return ''

        if not o:
            return ''

        return o

    def _find_val(self, output, word):
        lines = output.split('\n')
        for line in lines:
            tple = line.split(':')
            if not len(tple) > 1:
                continue

            name = tple[0].strip()
            val = ':'.join(tple[1:]).strip()

            if not name.lower() == word.lower():
                continue

            return val.strip()

        return ''

    def parse_smi_output(self, output):
        gpu_stat = GPUStatus()

        gpu_stat.product_name = self._find_val(output, 'Product Name')
        gpu_stat.pci_device_id = self._find_val(output, 'PCI Device/Vendor ID')
        gpu_stat.pci_location = self._find_val(output, 'PCI Location ID')
        gpu_stat.display = self._find_val(output, 'Display')
        gpu_stat.driver_version = self._find_val(output, 'Driver Version')

        temp_str = self._find_val(output, 'Temperature')
        if temp_str:
            temp, units = temp_str.split()
            gpu_stat.temperature = int(temp)

        fan_str = self._find_val(output, 'Fan Speed')
        if fan_str:
            # Fan speed in RPM
            fan_spd = float(fan_str.strip(r'\%').strip()) * 0.01 * MAX_FAN_RPM
            # Convert fan speed to Hz
            gpu_stat.fan_speed = self._rpm_to_rads(fan_spd)

        usage_str = self._find_val(output, 'GPU')
        if usage_str:
            usage = usage_str.strip(r'\%').strip()
            gpu_stat.gpu_usage = int(usage)

        mem_str = self._find_val(output, 'Memory')
        if mem_str:
            mem = mem_str.strip(r'\%').strip()
            gpu_stat.memory_usage = int(mem)

        return gpu_stat

    def gpu_status_to_diag(self, gpu_stat):
        stat = DiagnosticStatus()
        stat.name = 'GPU Status'
        stat.message = 'OK'
        stat.level = DiagnosticStatus.OK
        stat.hardware_id = gpu_stat.product_name

        stat.values.append(KeyValue(key='Product Name',
                           value=gpu_stat.product_name))
        # stat.values.append(KeyValue(key='PCI Device/Vendor ID', value=gpu_stat.pci_device_id))
        # stat.values.append(KeyValue(key='PCI Location ID', value=gpu_stat.pci_location))
        # stat.values.append(KeyValue(key='Display', value=gpu_stat.display))
        stat.values.append(KeyValue(key='Driver Version',
                           value=gpu_stat.driver_version))
        stat.values.append(KeyValue(key='Temperature (C)',
                           value='%.0f' % gpu_stat.temperature))
        stat.values.append(KeyValue(key='Fan Speed (RPM)',
                           value='%.0f' % self._rads_to_rpm(gpu_stat.fan_speed)))
        stat.values.append(
            KeyValue(key='Usage (%)', value='%.0f' % gpu_stat.gpu_usage))
        stat.values.append(
            KeyValue(key='Memory (%)', value='%.0f' % gpu_stat.memory_usage))

        self._usage_pub.publish(Float32(gpu_stat.gpu_usage * 1e-2))
        self._mem_usage_pub.publish(Float32(gpu_stat.memory_usage * 1e-2))

        # Check for valid data
        if not gpu_stat.product_name:
            stat.level = DiagnosticStatus.ERROR
            stat.message = 'No Device Data'
            return stat

        # Check load
        if gpu_stat.gpu_usage > 98:
            stat.level = max(stat.level, DiagnosticStatus.WARN)
            stat.message = 'High Load'

        # Check thresholds
        if gpu_stat.temperature > 90:
            stat.level = max(stat.level, DiagnosticStatus.WARN)
            stat.message = 'High Temperature'
        if gpu_stat.temperature > 95:
            stat.level = max(stat.level, DiagnosticStatus.ERROR)
            stat.message = 'Temperature Alarm'

        # Check fan
        if gpu_stat.fan_speed == 0:
            stat.level = max(stat.level, DiagnosticStatus.ERROR)
            stat.message = 'No Fan Speed'

        return stat


if __name__ == '__main__':
    rospy.init_node('gpu_monitor')

    monitor = GPUMonitor()
    my_rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        monitor.pub_status()
        my_rate.sleep()
