#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2024 gr-phaser author.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


import time
import numpy as np
from gnuradio import gr
import pmt
import adi
from typing import List


class blk(gr.sync_block):
    """
    docstring for block phaser_radar
    """
    def __init__(self, 
                 sdr_ip,
                 sample_rate: float,
                 sdr_freq: float,
                 # Rx params
                 rx_enabled_channels: List[int],
                 rx_gain: float,
                 rx_offset_samps: int,
                 # Tx params
                 tx_enabled_channels: List[int],
                 tx_hardwaregain_chan0: float,
                 tx_hardwaregain_chan1: float,
                 tx_cyclic_buffer: bool,
                 # Phaser params
                 phaser_uri: str,
                 phaser_output_freq: float,
                 phaser_chan_phase: List[float],
                 phaser_chan_gain: List[float],
                 # Radar params
                 radar_mode: str,
                 num_bursts: int,
                 pri: float,
                 fmcw_sweep_duration: float,
                 fmcw_if_freq: float,
                 fmcw_sweep_bandwidth: float,
                 fmcw_ramp_mode: str,
                 ):
        gr.sync_block.__init__(self,
            name="phaser_radar",
            in_sig=[],
            out_sig=[])
        # TODO: Set a variable number of input/output ports based on active channels
        self.message_port_register_in(pmt.intern("in"))
        self.message_port_register_out(pmt.intern("out"))
        self.set_msg_handler(pmt.intern("in"), self.handle_msg)
        self.tx_ready = False
        self.stop_called = False
        self.logger = gr.logger(self.alias())


        # Rx params
        self.sdr_ip = sdr_ip
        self.sample_rate = sample_rate
        self.sdr_freq = sdr_freq
        self.rx_enabled_channels = rx_enabled_channels
        self.rx_gain = rx_gain
        self.rx_offset_samps = rx_offset_samps
        # Tx params
        self.tx_enabled_channels = tx_enabled_channels
        self.tx_hardwaregain_chan0 = tx_hardwaregain_chan0
        self.tx_hardwaregain_chan1 = tx_hardwaregain_chan1
        self.tx_cyclic_buffer = tx_cyclic_buffer
        # Phaser params
        self.phaser_uri = phaser_uri
        self.phaser_output_freq = phaser_output_freq
        self.phaser_chan_phase = phaser_chan_phase
        self.phaser_chan_gain = phaser_chan_gain
        # Radar params
        self.radar_mode = radar_mode
        self.num_bursts = num_bursts
        self.pri = pri
        self.fmcw_sweep_duration = fmcw_sweep_duration
        self.fmcw_sweep_bandwidth = fmcw_sweep_bandwidth
        self.fmcw_if_freq = fmcw_if_freq
        self.fmcw_ramp_mode = fmcw_ramp_mode
        
        self.configure_hardware()

        self.main_thread = gr.threading.Thread(target=self.run)
        self.main_thread.start()

    def configure_hardware(self):
        self.sdr = adi.ad9361(uri=self.sdr_ip)
        self.phaser = adi.CN0566(uri=self.phaser_uri, sdr=self.sdr)

        # Configure Rx
        self.sdr.sample_rate = int(self.sample_rate)
        self.sdr.rx_lo = int(self.sdr_freq)
        self.sdr.rx_enabled_channels = self.rx_enabled_channels
        self.sdr.gain_control_mode_chan0 = 'manual'  # manual or slow_attack
        self.sdr.gain_control_mode_chan1 = 'manual'  # manual or slow_attack
        self.sdr.rx_hardwaregain_chan0 = int(self.rx_gain)   # must be between -3 and 70
        self.sdr.rx_hardwaregain_chan1 = int(self.rx_gain)   # must be between -3 and 70

        # Configure Tx
        self.sdr.tx_lo = int(self.sdr_freq)
        self.sdr.tx_enabled_channels = self.tx_enabled_channels
        self.sdr.tx_hardwaregain_chan0 = self.tx_hardwaregain_chan0
        self.sdr.tx_hardwaregain_chan1 = self.tx_hardwaregain_chan1
        self.sdr.tx_cyclic_buffer = self.tx_cyclic_buffer

        # Configure phaser
        self.phaser.configure(device_mode='rx')
        self.phaser.load_gain_cal()
        self.phaser.load_channel_cal()
        for i in range(0, 8):
            self.phaser.set_chan_phase(i, self.phaser_chan_phase[i])
            self.phaser.set_chan_gain(i, self.phaser_chan_gain[i], apply_cal=True)
        self.phaser._gpios.gpio_tx_sw = 0  # 0 = TX_OUT_2, 1 = TX_OUT_1
        self.phaser._gpios.gpio_vctrl_1 = 1 # 1=Use onboard PLL/LO source  (0=disable PLL and VCO, and set switch to use external LO input)
        self.phaser._gpios.gpio_vctrl_2 = 1 # 1=Send LO to transmit circuitry  (0=disable Tx path, and send LO to LO_OUT)
    
        self.sdr_pins = adi.one_bit_adc_dac(self.sdr_ip)
        time.sleep(0.1)
        self.sdr_pins.gpio_phaser_enable = True # when true, each channel[1] start outputs a pulse to Pluto L10P pin (TXDATA_1V8 on Phaser schematic)
        self.sdr_pins.gpio_tdd_ext_sync = True
        time.sleep(0.1)

        if self.radar_mode.lower() == 'fmcw':
            self.configure_fmcw()
        else:
            self.configure_pulsed()

    def configure_pulsed(self):
        self.sdr.rx_buffer_size = self.rx_offset_samps + round(self.pri * self.sample_rate * self.num_bursts)
        self.phaser.frequency = int((self.phaser_output_freq + self.sdr_freq) / 4)
            
        # Configure TDD
        tddn = adi.tddn(self.sdr_ip)
        tddn.enable = False
        frame_length_ms = self.pri * 1e3

        tddn.startup_delay_ms        = 0
        tddn.frame_length_ms         = frame_length_ms
        tddn.burst_count             = 0
        tddn.internal_sync_period_ms = 0

        tddn.channel[0].on_raw    = 0
        tddn.channel[0].off_raw  = 10
        tddn.channel[0].polarity = 0
        tddn.channel[0].enable   = 1

        # RX DMA SYNC
        tddn.channel[1].on_raw    = 0
        tddn.channel[1].off_raw  = 10
        tddn.channel[1].polarity = 0
        tddn.channel[1].enable   = 1

        # TX DMA SYNC
        tddn.channel[2].on_raw    = 0
        tddn.channel[2].off_raw  = 10
        tddn.channel[2].polarity = 0
        tddn.channel[2].enable   = 1

        tddn.sync_external = True  # enable external sync trigger
        tddn.sync_internal = False # enable the internal sync trigger
        tddn.sync_reset    = False # reset the internal counter when receiving a sync event
        tddn.enable        = True  # enable TDD engine

    def configure_fmcw(self):
        # Configure ADF4159 ramping PLL
        vco_freq = self.phaser_output_freq + self.sdr_freq + self.fmcw_if_freq
        num_steps = int(self.fmcw_sweep_duration * 1e6) # One step per us
        self.phaser.frequency = int(vco_freq / 4)
        self.phaser.freq_dev_range = int(self.fmcw_sweep_bandwidth / 4)
        self.phaser.freq_dev_step = int(self.phaser.freq_dev_range / num_steps)
        self.phaser.freq_dev_time = int(self.fmcw_sweep_duration * 1e6)
        self.phaser.dely_start_en = 0
        self.phaser.ramp_delay_en = 0
        self.phaser.trig_delay_en = 0
        self.phaser.ramp_mode = self.fmcw_ramp_mode
        self.phaser.sing_ful_tri = 0
        self.phaser.tx_trig_en = 1
        self.phaser.enable = 0

        # Configure TDD
        tddn = adi.tddn(self.sdr_ip)
        tddn.enable = False
        frame_length_ms = self.fmcw_sweep_duration * 1e3

        tddn.startup_delay_ms        = 0
        tddn.frame_length_ms         = frame_length_ms
        tddn.burst_count             = self.num_bursts
        tddn.internal_sync_period_ms = 0

        tddn.channel[0].on_raw    = 0
        tddn.channel[0].off_raw  = 10
        tddn.channel[0].polarity = 0
        tddn.channel[0].enable   = 1

        # RX DMA SYNC
        tddn.channel[1].on_raw    = 0
        tddn.channel[1].off_raw  = 10
        tddn.channel[1].polarity = 0
        tddn.channel[1].enable   = 1

        # TX DMA SYNC
        tddn.channel[2].on_raw    = 0
        tddn.channel[2].off_raw  = 10
        tddn.channel[2].polarity = 0
        tddn.channel[2].enable   = 1

        tddn.sync_external = True  # enable external sync trigger
        tddn.sync_internal = False # enable the internal sync trigger
        tddn.sync_reset    = False # reset the internal counter when receiving a sync event
        tddn.enable        = True  # enable TDD engine
        self.sdr.rx_buffer_size = self.rx_offset_samps + int(self.sample_rate * frame_length_ms*1e-3 * self.num_bursts)


        # IF Carrier
        N = self.sdr.rx_buffer_size
        fc = int(self.fmcw_if_freq)
        ts = 1 / float(self.sample_rate)
        t = np.arange(0, N * ts, ts)
        i = np.cos(2 * np.pi * t * fc)
        q = np.sin(2 * np.pi * t * fc)
        iq = 2**14 * (i + 1j*q)

        self.sdr._ctx.set_timeout(30000)
        self.sdr._rx_init_channels()
        self.sdr.tx([iq, iq])
        self.tx_ready = True

    
    def handle_msg(self, msg):
        data, meta = None, None
        if pmt.is_dict(msg):
            data, meta = None, msg
        elif pmt.is_vector(msg):
            data, meta = msg, None
        elif pmt.is_pair(msg):
            data, meta = pmt.car(msg), pmt.cdr(msg)
        else:
            self.logger.warn("Invalid message input: expected PMT dict, vector, or pair")
        if data is not None:
            if self.radar_mode.lower() == 'pulsed':
                # Update tx data
                iq = np.asarray(pmt.to_python(data)) * 2**14
                self.sdr.tx_destroy_buffer()
                self.sdr.tx([iq, iq])
                self.tx_ready = True
            else:
                self.logger.warn("Invalid message input: FMCW mode does not support arbitrary waveforms")
        if meta is not None:
            # TODO: Update phaser/self.sdr parameters
            pass

    def run(self):
        while True:
            if self.stop_called:
                return
            if self.tx_ready:
                self.phaser._gpios.gpio_burst = 0
                self.phaser._gpios.gpio_burst = 1
                self.phaser._gpios.gpio_burst = 0
                data = self.sdr.rx()
                sum_data = np.sum(np.array(data), axis=0).astype(np.complex64)
                sum_data = sum_data[self.rx_offset_samps:]
                pmt_out = pmt.cons(pmt.make_dict(), pmt.to_pmt(sum_data))
                self.message_port_pub(pmt.intern("out"), pmt_out)
                # TODO: In FMCW mode, lop of the first so many samples as in the video

    def stop(self):
        self.stop_called = True
        self.main_thread.join()

        # Gracefully shut down pluto
        self.sdr.tx_destroy_buffer()
        self.sdr.rx_destroy_buffer()
        self.sdr_pins.gpio_phaser_enable = False
        return True
    