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

import socket

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

hdd_level_warn = 0.95
hdd_level_error = 0.99
hdd_temp_warn = 55.0
hdd_temp_error = 70.0

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }
temp_dict = { 0: 'OK', 1: 'Hot', 2: 'Critical Hot' }
usage_dict = { 0: 'OK', 1: 'Low Disk Space', 2: 'Very Low Disk Space' }

REMOVABLE = ['/dev/sg1', '/dev/sdb'] # Store removable drives so we can ignore if removed


class hdd_monitor(Node):
    def __init__(self, hostname, diag_hostname):
        super().__init__('hdd_monitor_%s' % hostname)
        self._mutex = threading.Lock()

        self._hostname = hostname
        self.declare_parameter('no_hdd_temp', False)
        self.declare_parameter('no_hdd_temp_warn', False)
        self.declare_parameter('hdd_level_warn', hdd_level_warn)
        self.declare_parameter('hdd_level_error', hdd_level_error)
        self.declare_parameter('hdd_temp_warn', hdd_temp_warn)
        self.declare_parameter('hdd_temp_error', hdd_temp_error)
        self._no_temp = self.get_parameter('no_hdd_temp').get_parameter_value().bool_value
        self._no_temp_warn = self.get_parameter('no_hdd_temp_warn').get_parameter_value().bool_value
        self._hdd_level_warn = self.get_parameter('hdd_level_warn').get_parameter_value().double_value
        self._hdd_level_error = self.get_parameter('hdd_level_error').get_parameter_value().double_value
        self._hdd_temp_warn = self.get_parameter('hdd_temp_warn').get_parameter_value().double_value
        self._hdd_temp_error = self.get_parameter('hdd_temp_error').get_parameter_value().double_value

        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 100)

        self._last_publish_time = self.get_clock().now()

        self._last_temp_time = self.get_clock().now()
        self._temp_timer = None
        if not self._no_temp:
            pass
            # TODO: Implement this
            # self._temp_stat = DiagnosticStatus()
            # self._temp_stat.name = "HDD Temperature (%s)" % diag_hostname
            # self._temp_stat.level = DiagnosticStatus.ERROR
            # self._temp_stat.hardware_id = hostname
            # self._temp_stat.message = 'No Data'
            # self._temp_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data'),
            #                           KeyValue(key = 'Time Since Last Update', value = 'N/A') ]
            # self.check_temps()

        self._last_usage_time = self.get_clock().now()
        self._usage_timer = None
        self._usage_stat = DiagnosticStatus()
        self._usage_stat.level = DiagnosticStatus.ERROR
        self._usage_stat.hardware_id = hostname
        self._usage_stat.name = 'HDD Usage (%s)' % diag_hostname
        self._usage_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                    KeyValue(key = 'Time Since Last Update', value = 'N/A') ]
        self.check_disk_usage()

        self.create_timer(1.0, self.publish_stats)

    ## Connects to hddtemp daemon to get temp, HDD make.
    def get_hddtemp_data(self, hostname='localhost', port=7634):
        try:
            hdd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            hdd_sock.connect((hostname, port))
            sock_data = ''
            while True:
                newdat = hdd_sock.recv(1024)
                if len(newdat) == 0:
                    break
                sock_data = sock_data + newdat
            hdd_sock.close()

            sock_vals = sock_data.split('|')

            # Format of output looks like ' | DRIVE | MAKE | TEMP | '
            idx = 0

            drives = []
            makes = []
            temps = []
            while idx + 5 < len(sock_vals):
                this_drive = sock_vals[idx + 1]
                this_make = sock_vals[idx + 2]
                this_temp = sock_vals[idx + 3]

                # Sometimes we get duplicate makes if hard drives are mounted
                # to two different points
                if this_make in makes:
                    idx += 5
                    continue

                drives.append(this_drive)
                makes.append(this_make)
                temps.append(this_temp)

                idx += 5

            return True, drives, makes, temps
        except:
            self.get_logger().error(traceback.format_exc())
            return False, ['Exception'], [traceback.format_exc()], [0]


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
        if self._temp_timer:
            self._temp_timer.cancel()
            self._temp_timer = None

        if self._usage_timer:
            self._usage_timer.cancel()
            self._usage_timer = None

    def check_temps(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        diag_strs = [ KeyValue(key = 'Update Status', value = 'OK' ) ,
                      KeyValue(key = 'Time Since Last Update', value = '0' ) ]
        diag_level = DiagnosticStatus.OK
        diag_message = 'OK'

        temp_ok, drives, makes, temps = self.get_hddtemp_data()

        for index in range(0, len(drives)):
            temp = temps[index]

            self.get_logger().info(f'Got temp: {temp}')
            if not isinstance(temp, str):
                tmp_u = temp.decode("UTF-8")
            else:
                tmp_u = unicode(temp)

            if not tmp_u.isnumeric() and drives[index] not in REMOVABLE:
                temp_level = DiagnosticStatus.ERROR
                temp_ok = False
            elif not tmp_u.isnumeric() and drives[index] in REMOVABLE:
                temp_level = DiagnosticStatus.OK
                temp = "Removed"
            else:
                temp_level = DiagnosticStatus.OK
                if float(temp) >= self._hdd_temp_warn:
                    temp_level = DiagnosticStatus.WARN
                if float(temp) >= self._hdd_temp_error:
                    temp_level = DiagnosticStatus.ERROR

            diag_level = max(diag_level, temp_level)

            diag_strs.append(KeyValue(key = 'Disk %d Temperature Status' % index, value = temp_dict[int.from_bytes(temp_level)]))
            diag_strs.append(KeyValue(key = 'Disk %d Mount Pt.' % index, value = drives[index]))
            diag_strs.append(KeyValue(key = 'Disk %d Device ID' % index, value = makes[index]))
            diag_strs.append(KeyValue(key = 'Disk %d Temperature' % index, value = str(temp)+"DegC"))

        if not temp_ok:
            diag_level = DiagnosticStatus.ERROR

        with self._mutex:
            self._last_temp_time = self.get_clock().now()
            self._temp_stat.values = diag_strs
            self._temp_stat.level = diag_level

            # Give No Data message if we have no reading
            self._temp_stat.message = temp_dict[int.from_bytes(diag_level)]
            if not temp_ok:
                self._temp_stat.message = 'Error'

            if self._no_temp_warn and temp_ok:
                self._temp_stat.level = DiagnosticStatus.OK

            if rclpy.ok():
                self._temp_timer = threading.Timer(10.0, self.check_temps)
                self._temp_timer.start()
            else:
                self.cancel_timers()

    def check_disk_usage(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = '0' ) ]
        diag_level = DiagnosticStatus.OK
        diag_message = 'OK'

        try:
            p = subprocess.Popen(["df", "-Pht", "ext4"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if (retcode == 0 or retcode == 1):
                diag_vals.append(KeyValue(key = 'Disk Space Reading', value = 'OK'))
                if sys.version_info.major == 3:
                    stdout = stdout.decode('UTF-8')
                rows = stdout.split('\n')
                del rows[0]
                row_count = 0

                for row in rows:
                    if len(row.split()) < 2:
                        continue

                    if not isinstance(row.split()[0], str):
                        row.split()[0] = row.split()[0].decode("UTF-8")

                    if row.split()[0] == "none":
                        continue

                    row_count += 1
                    g_available = row.split()[-3]
                    g_use = row.split()[-2]
                    name = row.split()[0]
                    size = row.split()[1]
                    mount_pt = row.split()[-1]

                    hdd_usage = float(g_use.replace("%", ""))*1e-2
                    if (hdd_usage < self._hdd_level_warn):
                        level = DiagnosticStatus.OK
                    elif (hdd_usage < self._hdd_level_error):
                        level = DiagnosticStatus.WARN
                    else:
                        level = DiagnosticStatus.ERROR

                    diag_vals.append(KeyValue(
                            key = 'Disk %d Name' % row_count, value = name))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Size' % row_count, value = size))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Available' % row_count, value = g_available))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Use' % row_count, value = g_use))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Status' % row_count, value = stat_dict[int.from_bytes(level)]))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Mount Point' % row_count, value = mount_pt))

                    diag_level = max(diag_level, level)
                    diag_message = usage_dict[int.from_bytes(diag_level)]

            else:
                diag_vals.append(KeyValue(key = 'Disk Space Reading', value = 'Failed'))
                diag_level = DiagnosticStatus.ERROR
                diag_message = stat_dict[int.from_bytes(diag_level)]


        except:
            self.get_logger().error(traceback.format_exc())

            diag_vals.append(KeyValue(key = 'Disk Space Reading', value = 'Exception'))
            diag_vals.append(KeyValue(key = 'Disk Space Ex', value = traceback.format_exc()))

            diag_level = DiagnosticStatus.ERROR
            diag_message = stat_dict[int.from_bytes(diag_level)]

        # Update status
        with self._mutex:
            self._last_usage_time = self.get_clock().now()
            self._usage_stat.values = diag_vals
            self._usage_stat.message = diag_message
            self._usage_stat.level = diag_level

            if rclpy.ok():
                self._usage_timer = threading.Timer(5.0, self.check_disk_usage)
                self._usage_timer.start()
            else:
                self.cancel_timers()


    def publish_stats(self):
        with self._mutex:
            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()

            if not self._no_temp:
                self.update_status_stale(self._temp_stat, self._last_temp_time)
                msg.status.append(self._temp_stat)

            self.update_status_stale(self._usage_stat, self._last_usage_time)
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
        node = hdd_monitor(hostname, options.diag_hostname)
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
