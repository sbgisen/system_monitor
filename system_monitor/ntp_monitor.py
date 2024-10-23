#!/usr/bin/env python
############################################################################
#    Copyright (C) 2009, Willow Garage, Inc.                               #
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

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import sys
import rclpy
import rclpy.utilities
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
import socket
from subprocess import Popen, PIPE

import time

import re

NAME = 'ntp_monitor'

class NTPMonitor(Node):
    def __init__(self, offset=500., self_offset=500., diag_hostname=None, error_offset=5000000.):
        super().__init__(NAME)
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 100)
        self.hostname = socket.gethostname()
        if diag_hostname is None:
            diag_hostname = self.hostname

        self.declare_parameter('reference_host', 'ntp.ubuntu.com')
        self.declare_parameter('offset_tolerance', 500.0)
        self.declare_parameter('error_offset_tolerance', 5000000.0)
        self.ntp_hostname = self.get_parameter('reference_host').get_parameter_value().string_value
        self.offset = self.get_parameter('offset_tolerance').get_parameter_value().double_value
        self.error_offset = self.get_parameter('error_offset_tolerance').get_parameter_value().double_value

        self.stat = DiagnosticStatus()
        self.stat.level = DiagnosticStatus.OK
        self.stat.name = "NTP offset from "+ diag_hostname + " to " + self.ntp_hostname
        self.stat.message = "OK"
        self.stat.hardware_id = self.hostname
        self.stat.values = []

        self.create_timer(1.0, self.ntp_monitor)

    def ntp_monitor(self):
        for st, host, off in [(self.stat, self.ntp_hostname, self.offset)]:
            try:
                p = Popen(["ntpdate", "-q", host], stdout=PIPE, stdin=PIPE, stderr=PIPE)
                res = p.wait()
                (o, e) = p.communicate()
            except OSError as e:
                if e.errno == 4:
                    break  #ctrl-c interrupt
                else:
                    raise
            if (res == 0):
                if sys.version_info.major == 3:
                    o = o.decode("UTF-8")

                measured_offset = float(o.split(' ')[3]) * 1000000
                st.level = DiagnosticStatus.OK
                st.message = "OK"
                st.values = [
                    KeyValue(key="Offset (us)", value=str(measured_offset)),
                    KeyValue(key="Offset tolerance (us)", value=str(off)),
                    KeyValue(key="Offset tolerance (us) for Error", value=str(self.error_offset))
                ]

                if (abs(measured_offset) > off):
                    st.level = DiagnosticStatus.WARN
                    st.message = "NTP Offset Too High"
                if (abs(measured_offset) > self.error_offset):
                    st.level = DiagnosticStatus.ERROR
                    st.message = "NTP Offset Too High"

            else:
                st.level = DiagnosticStatus.ERROR
                st.message = "Error Running ntpdate. Returned %d" % res
                st.values = [
                    KeyValue("Offset (us)", "N/A"),
                    KeyValue("Offset tolerance (us)", str(off)),
                    KeyValue("Offset tolerance (us) for Error", str(self.error_offset)),
                    KeyValue("Output", o),
                    KeyValue("Errors", e)
                ]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [self.stat]
        self.pub.publish(msg)
        time.sleep(1)


def main() -> None:
    """Main function."""
    import optparse
    parser = optparse.OptionParser(usage="usage: ntp_monitor ntp-hostname []")
    parser.add_option("--offset-tolerance", dest="offset_tol",
                      action="store", default=500.0,
                      help="Offset from NTP host", metavar="OFFSET-TOL")
    parser.add_option("--error-offset-tolerance", dest="error_offset_tol",
                      action="store", default=5000000.0,
                      help="Offset from NTP host. Above this is error", metavar="OFFSET-TOL")
    parser.add_option("--self_offset-tolerance", dest="self_offset_tol",
                      action="store", default=500.0,
                      help="Offset from self", metavar="SELF_OFFSET-TOL")
    parser.add_option("--diag-hostname", dest="diag_hostname",
                      help="Computer name in diagnostics output (ex: 'c1')",
                      metavar="DIAG_HOSTNAME",
                      action="store", default=None)
    argv = rclpy.utilities.remove_ros_args()
    options, args = parser.parse_args(argv)
    rclpy.init()
    try:
        offset = int(options.offset_tol)
        self_offset = int(options.self_offset_tol)
        error_offset = int(options.error_offset_tol)
    except:
        parser.error("Offsets must be numbers")

    try:
        node = NTPMonitor(offset, self_offset, options.diag_hostname, error_offset)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
