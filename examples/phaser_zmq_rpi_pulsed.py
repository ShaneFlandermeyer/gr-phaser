#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
# GNU Radio version: v3.10.11.0-1-gee27d6f3

from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import zeromq
from gnuradio.phaser import phaser_radar
import numpy as np
import threading




class phaser_zmq_rpi_pulsed(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "Not titled yet", catch_exceptions=True)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 30e6
        self.angle = angle = 0

        ##################################################
        # Blocks
        ##################################################

        self.zeromq_sub_msg_source_0 = zeromq.sub_msg_source('tcp://169.254.227.151:2001', 100, True)
        self.zeromq_pub_msg_sink_0 = zeromq.pub_msg_sink('tcp://169.254.227.151:3001', 100, True)
        self.phaser_phaser_radar_0 = phaser_radar.blk(
          'ip:pluto.local',
          samp_rate,
          2.1e9,
          [0, 1],
          10,
          101,
          0,
          (-1),
          [0, 1],
          -88,
          -20,
          True,
          'ip:phaser.local',
          10e9,
          [0, 0, 0, 0, 0, 0, 0, 0],
          [127]*8,
          '',
          '',
          'pulsed',
          64,
          1/20e3,
          500e-6,
          100e3,
          500e6,
          'single_sawtooth_burst',
        )


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.phaser_phaser_radar_0, 'out'), (self.zeromq_pub_msg_sink_0, 'in'))
        self.msg_connect((self.zeromq_sub_msg_source_0, 'out'), (self.phaser_phaser_radar_0, 'in'))


    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate

    def get_angle(self):
        return self.angle

    def set_angle(self, angle):
        self.angle = angle




def main(top_block_cls=phaser_zmq_rpi_pulsed, options=None):
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()
    tb.flowgraph_started.set()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
