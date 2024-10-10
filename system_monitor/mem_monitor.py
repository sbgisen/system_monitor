#!/usr/bin/env python
############################################################################
#    Copyright (C) 2009, Willow Garage, Inc.                               #
#    Copyright (C) 2013 by Ralf Kaestner                                   #
#    ralf.kaestner@gmail.com                                               #
#    Copyright (C) 2013 by Jerome Maye                                     #
#    jerome.maye@mavt.ethz.ch                                              #
#                                                                          #
#    All rights reserved.                                                  #
#                                                                          #
#    Redistribution and use in source and binary forms, with or without    #
#    modification, are permitted provided that the following conditions    #
#    are met:                                                              #
#                                                                          #
#    1. Redistributions of source code must retain the above copyright     #
#       notice, this list of conditions and the following disclaimer.      #
#                                                                          #
#    2. Redistributions in binary form must reproduce the above copyright  #
#       notice, this list of conditions and the following disclaimer in    #
#       the documentation and/or other materials provided with the         #
#       distribution.                                                      #
#                                                                          #
#    3. The name of the copyright holders may be used to endorse or        #
#       promote products derived from this software without specific       #
#       prior written permission.                                          #
#                                                                          #
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   #
#    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     #
#    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     #
#    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        #
#    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  #
#    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  #
#    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      #
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      #
#    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    #
#    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN     #
#    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       #
#    POSSIBILITY OF SUCH DAMAGE.                                           #
############################################################################

from __future__ import with_statement

import rclpy
import rclpy.utilities
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
import traceback
import threading
from threading import Timer
import sys, os, time
from time import sleep
import subprocess
import string

import socket

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

mem_level_warn = 0.95
mem_level_error = 0.99

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }


class MemMonitor(Node):
    def __init__(self, hostname, diag_hostname):
        super().__init__('mem_monitor_%s' % hostname)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 100)

        self._mutex = threading.Lock()

        self.declare_parameter('mem_level_warn', mem_level_warn)
        self.declare_parameter('mem_level_error', mem_level_error)
        self._mem_level_warn = self.get_parameter('mem_level_warn').get_parameter_value().double_value
        self._mem_level_error = self.get_parameter('mem_level_error').get_parameter_value().double_value

        self._usage_timer = None

        self._usage_stat = DiagnosticStatus()
        self._usage_stat.name = 'Memory Usage (%s)' % diag_hostname
        self._usage_stat.level = DiagnosticStatus.WARN
        self._usage_stat.hardware_id = hostname
        self._usage_stat.message = 'No Data'
        self._usage_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                    KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._last_usage_time = self.get_clock().now()
        self._last_publish_time = self.get_clock().now()

        # Start checking everything
        self.check_usage()

        self.create_timer(1.0, self.publish_stats)

    def update_status_stale(self, stat, last_update_time):
        time_since_update = self.get_clock().now() - last_update_time

        stale_status = 'OK'
        if time_since_update > Duration(seconds=20) and time_since_update <= Duration(seconds=35):
            stale_status = 'Lagging'
            if stat.level == DiagnosticStatus.OK:
                stat.message = stale_status
            elif stat.message.find(stale_status) < 0:
                stat.message = ', '.join([stat.message, stale_status])
            stat.level = max(stat.level, DiagnosticStatus.WARN)
        if time_since_update > Duration(seconds=35):
            stale_status = 'Stale'
            if stat.level == DiagnosticStatus.OK:
                stat.message = stale_status
            elif stat.message.find(stale_status) < 0:
                stat.message = ', '.join([stat.message, stale_status])
            stat.level = max(stat.level, DiagnosticStatus.ERROR)

        stat.values.pop(0)
        stat.values.pop(0)
        stat.values.insert(0, KeyValue(key='Update Status', value=stale_status))
        stat.values.insert(1, KeyValue(key='Time Since Update', value=str(time_since_update)))

    ## Must have the lock to cancel everything
    def cancel_timers(self):
        if self._usage_timer:
            self._usage_timer.cancel()

    def check_memory(self):
        values = []
        level = DiagnosticStatus.OK
        msg = ''

        mem_dict = { 0: 'OK', 1: 'Low Memory', 2: 'Very Low Memory' }

        try:
            p = subprocess.Popen('free -tm',
                                stdout = subprocess.PIPE,
                                stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                values.append(KeyValue(key = "\"free -tm\" Call Error", value = str(retcode)))
                return DiagnosticStatus.ERROR, values

            if sys.version_info.major == 3:
                stdout = stdout.decode('UTF-8')

            rows = stdout.split('\n')
            data = rows[1].split()
            total_mem_physical = data[1]
            used_mem_physical = data[2]
            free_mem_physical = data[3]
            data = rows[1].split()
            used_mem_wo_buffers = data[5]
            free_mem_wo_buffers = data[6]
            data = rows[2].split()
            total_mem_swap = data[1]
            used_mem_swap = data[2]
            free_mem_swap = data[3]
            data = rows[3].split()
            total_mem = data[1]
            used_mem = data[2]
            free_mem = data[3]

            level = DiagnosticStatus.OK
            mem_usage = float(used_mem_wo_buffers)/float(total_mem_physical)
            if (mem_usage < self._mem_level_warn):
                level = DiagnosticStatus.OK
            elif (mem_usage < self._mem_level_error):
                level = DiagnosticStatus.WARN
            else:
                level = DiagnosticStatus.ERROR

            values.append(KeyValue(key = 'Memory Status', value = mem_dict[int.from_bytes(level)]))
            values.append(KeyValue(key = 'Total Memory (Physical)', value = total_mem_physical+"M"))
            values.append(KeyValue(key = 'Used Memory (Physical)', value = used_mem_physical+"M"))
            values.append(KeyValue(key = 'Free Memory (Physical)', value = free_mem_physical+"M"))
            values.append(KeyValue(key = 'Used Memory (Physical w/o Buffers)', value = used_mem_wo_buffers+"M"))
            values.append(KeyValue(key = 'Free Memory (Physical w/o Buffers)', value = free_mem_wo_buffers+"M"))
            values.append(KeyValue(key = 'Total Memory (Swap)', value = total_mem_swap+"M"))
            values.append(KeyValue(key = 'Used Memory (Swap)', value = used_mem_swap+"M"))
            values.append(KeyValue(key = 'Free Memory (Swap)', value = free_mem_swap+"M"))
            values.append(KeyValue(key = 'Total Memory', value = total_mem+"M"))
            values.append(KeyValue(key = 'Used Memory', value = used_mem+"M"))
            values.append(KeyValue(key = 'Free Memory', value = free_mem+"M"))

            msg = mem_dict[int.from_bytes(level)]
        except Exception as e:
            self.get_logger().error(traceback.format_exc())
            msg = 'Memory Usage Check Error'
            values.append(KeyValue(key = msg, value = str(e)))
            level = DiagnosticStatus.ERROR

        return level, mem_dict[int.from_bytes(level)], values

    def check_usage(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        diag_level = DiagnosticStatus.OK
        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = 0 )]
        diag_msgs = []

        # Check memory
        mem_level, mem_msg, mem_vals = self.check_memory()
        diag_vals.extend(mem_vals)
        if mem_level > DiagnosticStatus.WARN:
            diag_msgs.append(mem_msg)
        diag_level = max(diag_level, mem_level)

        if diag_msgs and diag_level > DiagnosticStatus.OK:
            usage_msg = ', '.join(set(diag_msgs))
        else:
            usage_msg = stat_dict[int.from_bytes(diag_level)]

        # Update status
        with self._mutex:
            self._last_usage_time = self.get_clock().now()
            self._usage_stat.level = diag_level
            self._usage_stat.values = diag_vals

            self._usage_stat.message = usage_msg

            if rclpy.ok():
                self._usage_timer = threading.Timer(5.0, self.check_usage)
                self._usage_timer.start()
            else:
                self.cancel_timers()

    def publish_stats(self):
        with self._mutex:
            # Update everything with last update times
            self.update_status_stale(self._usage_stat, self._last_usage_time)

            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.status.append(self._usage_stat)

            if self.get_clock().now() - self._last_publish_time > Duration(seconds=0.5):
                self._diag_pub.publish(msg)
                self._last_publish_time = self.get_clock().now()


def main() -> None:
    """Main function."""
    hostname = socket.gethostname()
    hostname = hostname.replace('-', '_')

    import optparse
    parser = optparse.OptionParser(usage="usage: cpu_monitor.py [--diag-hostname=cX]")
    parser.add_option("--diag-hostname",
                      dest="diag_hostname",
                      help="Computer name in diagnostics output (ex: 'c1')",
                      metavar="DIAG_HOSTNAME",
                      action="store",
                      default=hostname)
    argv = rclpy.utilities.remove_ros_args()
    options, args = parser.parse_args(argv)
    rclpy.init()
    try:
        node = MemMonitor(hostname, options.diag_hostname)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        node.cancel_timers()
        node.destroy_node()


if __name__ == '__main__':
    main()

