#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2024 gr-phaser author.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


import numpy
from gnuradio import gr
import pmt
import adi

class pluto_radar(gr.sync_block):
    """
    docstring for block pluto_radar
    """
    def __init__(self, 
                 uri, 
                 sample_rate, 
                 center_freq, 
                 rx_gain, 
                 rx_offset_samples, 
                 burst_start_samples, 
                 burst_stop_samples, 
                 tx_gain, 
                 tx_cyclic_buffer, 
                 radar_mode,
                 num_bursts, 
                 pri):
        gr.sync_block.__init__(self,
                            name="phaser_radar",
                            in_sig=[],
                            out_sig=[])
        self.message_port_register_in(pmt.intern("in"))
        self.message_port_register_out(pmt.intern("out"))
        self.set_msg_handler(pmt.intern("in"), self.handle_msg)

    def handle_msg(self, msg):
        print("Received message")
        print(msg)
        self.message_port_pub(pmt.intern("out"), pmt.intern("out"))