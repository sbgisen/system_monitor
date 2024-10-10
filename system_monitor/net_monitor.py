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
import re

import socket

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

net_level_warn = 0.95
net_capacity = 128.0

stat_dict = {0: 'OK', 1: 'Warning', 2: 'Error'}

def get_sys_net_stat(iface, syst):
    cmd = 'cat /sys/class/net/%s/statistics/%s' %(iface, syst)
    p = subprocess.Popen(cmd,
                         stdout = subprocess.PIPE,
                         stderr = subprocess.PIPE, shell = True)
    stdout, stderr = p.communicate()
    if sys.version_info.major == 3:
        stdout = stdout.decode("UTF-8")
    return (p.returncode, stdout.strip())

def get_sys_net(iface, syst):
    cmd = 'cat /sys/class/net/%s/%s' %(iface, syst)
    p = subprocess.Popen(cmd,
                         stdout = subprocess.PIPE,
                         stderr = subprocess.PIPE, shell = True)
    stdout, stderr = p.communicate()
    if sys.version_info.major == 3:
        stdout = stdout.decode("UTF-8")
    return (p.returncode, stdout.strip())

class NetMonitor(Node):
    def __init__(self, hostname, diag_hostname):
        super().__init__('net_monitor_%s' % hostname)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 100)
        self._mutex = threading.Lock()
        self.declare_parameter('net_level_warn', net_level_warn)
        self.declare_parameter('net_capacity', net_capacity)
        self._net_level_warn = self.get_parameter('net_level_warn').get_parameter_value().double_value
        self._net_capacity = self.get_parameter('net_capacity').get_parameter_value().double_value
        self._usage_timer = None
        self._usage_stat = DiagnosticStatus()
        self._usage_stat.name = 'Network Usage (%s)' % diag_hostname
        self._usage_stat.level = DiagnosticStatus.WARN
        self._usage_stat.hardware_id = hostname
        self._usage_stat.message = 'No Data'
        self._usage_stat.values = [KeyValue(key = 'Update Status',
                                   value = 'No Data' ),
                                   KeyValue(key = 'Time Since Last Update',
                                   value = 'N/A') ]
        self._last_usage_time = self.get_clock().now()
        self._last_publish_time = self.get_clock().now()
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

    def cancel_timers(self):
        if self._usage_timer:
            self._usage_timer.cancel()

    def check_network(self):
        values = []
        net_dict = {0: 'OK', 1: 'High Network Usage', 2: 'Network Down', 3: 'Call Error'}
        try:
            p = subprocess.Popen('ifstat -q -S 1 1',
                                 stdout = subprocess.PIPE,
                                 stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode
            if retcode == 3:
                values.append(KeyValue(key = "\"ifstat -q -S 1 1\" Call Error",
                  value = str(retcode)))
                return DiagnosticStatus.ERROR, net_dict[3], values

            if sys.version_info.major == 3:
                stdout = stdout.decode("UTF-8")
            rows = stdout.split('\n')
            data = rows[0].split()
            ifaces = []
            for i in range(0, len(data)):
                ifaces.append(data[i])
            data = rows[2].split()
            kb_in = []
            kb_out = []
            for i in range(0, len(data), 2):
                kb_in.append(data[i])
                kb_out.append(data[i + 1])
            level = DiagnosticStatus.OK
            for i in range(0, len(ifaces)):
                values.append(KeyValue(key = 'Interface Name',
                  value = ifaces[i]))
                (retcode, cmd_out) = get_sys_net(ifaces[i], 'operstate')
                if retcode == 0:
                    values.append(KeyValue(key = 'State', value = cmd_out))
                    ifacematch = re.match('eth[0-9]+', ifaces[i])
                    if ifacematch and (cmd_out == 'down' or cmd_out == 'dormant'):
                        level = DiagnosticStatus.ERROR
                values.append(KeyValue(key = 'Input Traffic',
                  value = str(float(kb_in[i]) / 1024) + " (MB/s)"))
                values.append(KeyValue(key = 'Output Traffic',
                  value = str(float(kb_out[i]) / 1024) + " (MB/s)"))
                net_usage_in = float(kb_in[i]) / 1024 / self._net_capacity
                net_usage_out = float(kb_out[i]) / 1024 / self._net_capacity
                if net_usage_in > self._net_level_warn or\
                  net_usage_out > self._net_level_warn:
                    level = DiagnosticStatus.WARN
                (retcode, cmd_out) = get_sys_net(ifaces[i], 'mtu')
                if retcode == 0:
                    values.append(KeyValue(key = 'MTU', value = cmd_out))
                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'rx_bytes')
                if retcode == 0:
                    values.append(KeyValue(key = 'Total received MB',
                      value = str(float(cmd_out) / 1024 / 1024)))
                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'tx_bytes')
                if retcode == 0:
                    values.append(KeyValue(key = 'Total transmitted MB',
                      value = str(float(cmd_out) / 1024 / 1024)))
                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'collisions')
                if retcode == 0:
                    values.append(KeyValue(key = 'Collisions', value = cmd_out))
                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'rx_errors')
                if retcode == 0:
                    values.append(KeyValue(key = 'Rx Errors', value = cmd_out))
                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'tx_errors')
                if retcode == 0:
                    values.append(KeyValue(key = 'Tx Errors', value = cmd_out))
        except Exception as e:
            self.get_logger().error(traceback.format_exc())
            msg = 'Network Usage Check Error'
            values.append(KeyValue(key = msg, value = str(e)))
            level = DiagnosticStatus.ERROR
        return level, net_dict[int.from_bytes(level)], values

    def check_usage(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return
        diag_level = DiagnosticStatus.OK
        diag_vals = [KeyValue(key = 'Update Status', value = 'OK'),
                     KeyValue(key = 'Time Since Last Update', value = 0)]
        diag_msgs = []
        net_level, net_msg, net_vals = self.check_network()
        diag_vals.extend(net_vals)
        if net_level > DiagnosticStatus.OK:
            diag_msgs.append(net_msg)
        diag_level = max(diag_level, net_level)
        if diag_msgs and diag_level > DiagnosticStatus.OK:
            usage_msg = ', '.join(set(diag_msgs))
        else:
            usage_msg = stat_dict[int.from_bytes(diag_level)]
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
        node = NetMonitor(hostname, options.diag_hostname)
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
