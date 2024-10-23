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

cpu_load_warn = 0.9
cpu_load_error = 1.1
cpu_load1_warn = 0.9
cpu_load5_warn = 0.8
cpu_temp_warn = 85.0
cpu_temp_error = 90.0

num_cores = subprocess.Popen('lscpu | grep "^CPU(s):"',
                                stdout= subprocess.PIPE,
                                stderr= subprocess.PIPE, shell=True )
try:
    num_cores = num_cores.communicate()[0].decode()
    num_cores = num_cores[-3]+num_cores[-2]
    num_cores = int(num_cores)
except:
    num_cores = int(num_cores.communicate()[0][-2])

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }


class CPUMonitor(Node):
    def __init__(self, hostname, diag_hostname):
        super().__init__('cpu_monitor_%s' % hostname)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 100)

        self._mutex = threading.Lock()

        self.declare_parameter('check_core_temps', True)
        self.declare_parameter('cpu_load_warn', cpu_load_warn)
        self.declare_parameter('cpu_load_error', cpu_load_error)
        self.declare_parameter('cpu_load1_warn', cpu_load1_warn)
        self.declare_parameter('cpu_load5_warn', cpu_load5_warn)
        self.declare_parameter('cpu_temp_warn', cpu_temp_warn)
        self.declare_parameter('cpu_temp_error', cpu_temp_error)
        self.declare_parameter('num_cores', num_cores)

        self._check_core_temps = self.get_parameter('check_core_temps').get_parameter_value().bool_value
        self._cpu_load_warn = self.get_parameter('cpu_load_warn').get_parameter_value().double_value
        self._cpu_load_error = self.get_parameter('cpu_load_error').get_parameter_value().double_value
        self._cpu_load1_warn = self.get_parameter('cpu_load1_warn').get_parameter_value().double_value
        self._cpu_load5_warn = self.get_parameter('cpu_load5_warn').get_parameter_value().double_value
        self._cpu_temp_warn = self.get_parameter('cpu_temp_warn').get_parameter_value().double_value
        self._cpu_temp_error = self.get_parameter('cpu_temp_error').get_parameter_value().double_value
        self._num_cores = self.get_parameter('num_cores').get_parameter_value().integer_value

        self._temps_timer = None
        self._usage_timer = None

        # Get temp_input files
        self._temp_vals = self.get_core_temp_names()

        # CPU stats
        self._temp_stat = DiagnosticStatus()
        self._temp_stat.name = 'CPU Temperature (%s)' % diag_hostname
        self._temp_stat.level = DiagnosticStatus.WARN
        self._temp_stat.hardware_id = hostname
        self._temp_stat.message = 'No Data'
        self._temp_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                   KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._usage_stat = DiagnosticStatus()
        self._usage_stat.name = 'CPU Usage (%s)' % diag_hostname
        self._usage_stat.level = DiagnosticStatus.WARN
        self._usage_stat.hardware_id = hostname
        self._usage_stat.message = 'No Data'
        self._usage_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                    KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._last_temp_time = self.get_clock().now()
        self._last_usage_time = self.get_clock().now()
        self._last_publish_time = self.get_clock().now()

        self._usage_old = 0
        self._has_warned_mpstat = False
        self._has_error_core_count = False

        # Start checking everything
        self.check_temps()
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

    # Restart temperature checking
    def _restart_temp_check(self):
        self.get_logger().error('Restarting temperature check thread in cpu_monitor. This should not happen')
        try:
            with self._mutex:
                if self._temps_timer:
                    self._temps_timer.cancel()

            self.check_temps()
        except Exception as e:
            self.get_logger().error('Unable to restart temp thread. Error: %s' % traceback.format_exc())


    ## Must have the lock to cancel everything
    def cancel_timers(self):
        if self._temps_timer:
            self._temps_timer.cancel()

        if self._usage_timer:
            self._usage_timer.cancel()

    ##\brief Check CPU core temps
    ##
    ## Use 'find /sys -name temp1_input' to find cores
    ## Read from every core, divide by 1000
    def check_core_temps(self, sys_temp_strings):
        diag_vals = []
        diag_level = DiagnosticStatus.OK
        diag_msgs = []

        for index, temp_str in enumerate(sys_temp_strings):
            if len(temp_str) < 5:
                continue

            cmd = 'cat %s' % temp_str
            p = subprocess.Popen(cmd, stdout = subprocess.PIPE,
                                stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                continue

            tmp = stdout.strip()
            if not isinstance(tmp, str):
                tmp_u = tmp.decode("UTF-8")
            else:
                tmp_u = unicode(tmp)

            if tmp_u.isnumeric():
                temp = float(tmp) / 1000
                diag_vals.append(KeyValue(key = 'Core %d Temperature' % index, value = str(temp)+"DegC"))

                if temp >= self._cpu_temp_warn:
                    diag_level = max(diag_level, DiagnosticStatus.WARN)
                    diag_msgs.append('Warm')
                elif temp >= self._cpu_temp_error:
                    diag_level = max(diag_level, DiagnosticStatus.ERROR)
                    diag_msgs.append('Hot')
            else:
                diag_level = max(diag_level, DiagnosticStatus.ERROR) # Error if not numeric value
                diag_vals.append(KeyValue(key = 'Core %s Temperature' % index, value = tmp))

        if not diag_vals:
            diag_level = DiagnosticStatus.ERROR
            diag_msgs = [ 'Core Temperature Error' ]
            diag_vals = [ KeyValue(key = 'Core Temperature Error', value = 'Cannot Read Core Temperature') ]

        return diag_vals, diag_msgs, diag_level

    ## Checks clock speed from reading from CPU info
    def check_clock_speed(self):
        vals = []
        msgs = []
        lvl = DiagnosticStatus.OK

        try:
            p = subprocess.Popen('cat /proc/cpuinfo | grep MHz',
                                stdout = subprocess.PIPE,
                                stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                lvl = DiagnosticStatus.ERROR
                msgs = [ 'Clock speed error' ]
                vals = [ KeyValue(key = 'Clock speed error', value = stderr),
                        KeyValue(key = 'Output', value = stdout) ]

                return (vals, msgs, lvl)

            if sys.version_info.major == 3:
                stdout = stdout.decode('UTF-8')

            for index, ln in enumerate(stdout.split('\n')):
                words = ln.split(':')
                if len(words) < 2:
                    continue

                speed = words[1].strip().split('.')[0] # Conversion to float doesn't work with decimal
                vals.append(KeyValue(key = 'Core %d Clock Speed' % index, value = speed+"MHz"))

        except Exception as e:
            self.get_logger().error('Exception in check_clock_speed: %s' % traceback.format_exc())
            lvl = DiagnosticStatus.ERROR
            msgs.append('Exception')
            vals.append(KeyValue(key = 'Exception', value = traceback.format_exc()))

        return vals, msgs, lvl


    # Add msgs output, too
    ##\brief Uses 'uptime' to see load average
    def check_uptime(self):
        level = DiagnosticStatus.OK
        vals = []

        load_dict = { 0: 'OK', 1: 'High Load', 2: 'Very High Load' }

        try:
            p = subprocess.Popen('uptime', stdout = subprocess.PIPE,
                                stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                vals.append(KeyValue(key = 'uptime Failed', value = stderr))
                return DiagnosticStatus.ERROR, vals

            if sys.version_info.major == 3:
                stdout = stdout.decode('UTF-8')

            upvals = stdout.split()
            load1 = float(upvals[-3].rstrip(',').replace(',','.'))/self._num_cores
            load5 = float(upvals[-2].rstrip(',').replace(',','.'))/self._num_cores
            load15 = float(upvals[-1].replace(',','.'))/self._num_cores

            # Give warning if we go over load limit
            if load1 > self._cpu_load1_warn or load5 > self._cpu_load5_warn:
                level = DiagnosticStatus.WARN

            vals.append(KeyValue(key = 'Load Average Status', value = load_dict[int.from_bytes(level)]))
            vals.append(KeyValue(key = 'Load Average (1min)', value = str(load1*1e2)+"%"))
            vals.append(KeyValue(key = 'Load Average (5min)', value = str(load5*1e2)+"%"))
            vals.append(KeyValue(key = 'Load Average (15min)', value = str(load15*1e2)+"%"))

        except Exception as e:
            self.get_logger().error('Exception in check_uptime: %s' % traceback.format_exc())
            level = DiagnosticStatus.ERROR
            vals.append(KeyValue(key = 'Load Average Status', value = traceback.format_exc()))

        return level, load_dict[int.from_bytes(level)], vals

    ##\brief Use mpstat to find CPU usage
    ##
    def check_mpstat(self):
        vals = []
        mp_level = DiagnosticStatus.OK

        load_dict = { 0: 'OK', 1: 'High Load', 2: 'Error' }
        try:
            p = subprocess.Popen('mpstat -P ALL 1 1',
                                stdout = subprocess.PIPE,
                                stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            try:
                stdout = stdout.decode()
            except:
                stdout = stdout
            retcode = p.returncode
            if retcode != 0:
                if not self._has_warned_mpstat:
                    self.get_logger().error('mpstat failed to run for cpu_monitor. Return code %d.', retcode)
                    self._has_warned_mpstat = True

                mp_level = DiagnosticStatus.ERROR
                vals.append(KeyValue(key = '\"mpstat\" Call Error', value = str(retcode)))
                return mp_level, 'Unable to Check CPU Usage', vals

            # Check which column '%idle' is, #4539
            # mpstat output changed between 8.06 and 8.1
            rows = stdout.split('\n')
            col_names = rows[2].split()

            idle_col = -1 if (len(col_names) > 2 and col_names[-1] == '%idle') else -2
            num_cores = 0
            cores_loaded = 0
            for index, row in enumerate(stdout.split('\n')):
                if index < 3:
                    continue

                # Skip row containing 'all' data
                if row.find('all') > -1:
                    continue
                lst = row.split()
                if len(lst) < 8:
                    continue

                ## Ignore 'Average: ...' data
                if lst[0].startswith('Average') or lst[0].startswith('Media'):
                    continue

                cpu_name = '%d' % (num_cores)
                idle = lst[idle_col]
                user = lst[2].replace(",",".")
                nice = lst[3].replace(",",".")
                system = lst[4].replace(",",".")

                core_level = DiagnosticStatus.OK
                usage = (float(user)+float(nice))*1e-2
                if usage > 10.0: # wrong reading, use old reading instead
                    self.get_logger().warn('Read CPU usage of %f percent. Reverting to previous reading of %f percent', usage, self._usage_old)
                    usage = self._usage_old
                self._usage_old = usage

                if usage >= self._cpu_load_warn:
                    cores_loaded += 1
                    core_level = DiagnosticStatus.WARN
                elif usage >= self._cpu_load_error:
                    core_level = DiagnosticStatus.ERROR

                vals.append(KeyValue(key = 'Core %s Status' % cpu_name, value = load_dict[int.from_bytes(core_level)]))
                vals.append(KeyValue(key = 'Core %s User' % cpu_name, value = user+"%"))
                vals.append(KeyValue(key = 'Core %s Nice' % cpu_name, value = nice+"%"))
                vals.append(KeyValue(key = 'Core %s System' % cpu_name, value = system+"%"))
                vals.append(KeyValue(key = 'Core %s Idle' % cpu_name, value = idle+"%"))

                num_cores += 1

            # Warn for high load only if we have <= 2 cores that aren't loaded
            if num_cores - cores_loaded <= 2 and num_cores > 2:
                mp_level = DiagnosticStatus.WARN

            if not self._num_cores:
                self._num_cores = num_cores

            # Check the number of cores if self._num_cores > 0, #4850
            if self._num_cores != num_cores:
                mp_level = DiagnosticStatus.WARN
                if not self._has_error_core_count:
                    self.get_logger().warn('Error checking number of cores. Expected %d, got %d. Computer may have not booted properly.', self._num_cores, num_cores)
                    self._has_error_core_count = True
                self._num_cores = num_cores
                return DiagnosticStatus.WARN, 'Incorrect number of CPU cores', vals

        except Exception as e:
            mp_level = DiagnosticStatus.ERROR
            vals.append(KeyValue(key = 'mpstat Exception', value = str(e)))

        return mp_level, load_dict[int.from_bytes(mp_level)], vals

    ## Returns names for core temperature files
    ## Returns list of names, each name can be read like file
    def get_core_temp_names(self):
        temp_vals = []
        try:
            p = subprocess.Popen('find /sys/devices -name temp1_input',
                                stdout = subprocess.PIPE,
                                stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if sys.version_info.major == 3:
                stdout = stdout.decode('UTF-8')

            if retcode != 0:
                self.get_logger().error('Error finding core temp locations: %s' % stderr)
                return []

            for ln in stdout.split('\n'):
                temp_vals.append(ln.strip())

            return temp_vals
        except:
            self.get_logger().error('Exception finding temp vals: %s' % traceback.format_exc())
            return []

    ## Call every 10sec at minimum
    def check_temps(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = str(0) ) ]
        diag_msgs = []
        diag_level = DiagnosticStatus.OK

        if self._check_core_temps:
            core_vals, core_msgs, core_level = self.check_core_temps(self._temp_vals)
            diag_vals.extend(core_vals)
            diag_msgs.extend(core_msgs)
            self.get_clock().sleep_for(Duration(seconds=1))
            diag_level = max(diag_level, core_level)

        diag_log = set(diag_msgs)
        if len(diag_log) > 0:
            message = ', '.join(diag_log)
        else:
            message = stat_dict[int.from_bytes(diag_level)]

        with self._mutex:
            self._last_temp_time = self.get_clock().now()

            self._temp_stat.level = diag_level
            self._temp_stat.message = message
            self._temp_stat.values = diag_vals

            if rclpy.ok():
                self._temps_timer = threading.Timer(5.0, self.check_temps)
                self._temps_timer.start()
            else:
                self.cancel_timers()

    def check_usage(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        diag_level = DiagnosticStatus.OK
        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = 0 )]
        diag_msgs = []

        # Check clock speed
        clock_vals, clock_msgs, clock_level = self.check_clock_speed()
        diag_vals.extend(clock_vals)
        diag_msgs.extend(clock_msgs)
        diag_level = max(diag_level, clock_level)

        # Check mpstat
        mp_level, mp_msg, mp_vals = self.check_mpstat()
        diag_vals.extend(mp_vals)
        if mp_level > DiagnosticStatus.OK:
            diag_msgs.append(mp_msg)
        self.get_clock().sleep_for(Duration(seconds=1))
        diag_level = max(diag_level, mp_level)

        # Check uptime
        uptime_level, up_msg, up_vals = self.check_uptime()
        diag_vals.extend(up_vals)
        if uptime_level > DiagnosticStatus.OK:
            diag_msgs.append(up_msg)
        diag_level = max(diag_level, uptime_level)

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
                self._usage_timer = threading.Timer(2.0, self.check_usage)
                self._usage_timer.start()
            else:
                self.cancel_timers()

    def publish_stats(self):
        with self._mutex:
            # Update everything with last update times
            self.update_status_stale(self._temp_stat, self._last_temp_time)
            self.update_status_stale(self._usage_stat, self._last_usage_time)

            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.status.append(self._temp_stat)
            msg.status.append(self._usage_stat)

            if self.get_clock().now() - self._last_publish_time > Duration(seconds=0.5):
                self._diag_pub.publish(msg)
                self._last_publish_time = self.get_clock().now()


        # Restart temperature checking if it goes stale, #4171
        # Need to run this without mutex
        if self.get_clock().now() - self._last_temp_time > Duration(seconds=90):
            self._restart_temp_check()


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
        node = CPUMonitor(hostname, options.diag_hostname)
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
