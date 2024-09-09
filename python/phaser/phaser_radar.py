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
from typing import List, Optional
import scipy


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
               rx_offset_samples: int,
               burst_start_sample: int,
               burst_stop_sample: int,
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
               gain_cal_file: str,
               phase_cal_file: str,
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
    self.message_port_register_in(pmt.intern("in"))
    self.message_port_register_out(pmt.intern("out"))
    self.set_msg_handler(pmt.intern("in"), self.handle_msg)
    # Set phaser metadata (for loop across init args)
    self.meta = pmt.make_dict()
    for key, val in locals().items():
      if key == "self":
        continue
      key = f"phaser:{key}"
      self.meta = pmt.dict_add(self.meta, pmt.intern(key), pmt.to_pmt(val))

    self.started = False
    self.stopped = False
    self.logger = gr.logger(self.alias())

    # Rx params
    self.sdr_ip = sdr_ip
    self.sample_rate = sample_rate
    self.sdr_freq = sdr_freq
    self.rx_enabled_channels = rx_enabled_channels
    self.rx_gain = rx_gain
    self.rx_offset_samples = rx_offset_samples
    if burst_start_sample == -1:
      self.burst_start_sample = None
    else:
      self.burst_start_sample = burst_start_sample
    if burst_stop_sample == -1:
      self.burst_stop_sample = None
    else:
      self.burst_stop_sample = burst_stop_sample
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
    self.gain_cal_file = gain_cal_file
    self.phase_cal_file = phase_cal_file
    # Radar params
    self.radar_mode = radar_mode
    self.num_bursts = num_bursts
    self.pri = pri
    self.fmcw_sweep_duration = fmcw_sweep_duration
    self.fmcw_sweep_bandwidth = fmcw_sweep_bandwidth
    self.fmcw_if_freq = fmcw_if_freq
    self.fmcw_ramp_mode = fmcw_ramp_mode
    # Derived parameters
    self.num_beams = len(self.rx_enabled_channels)
    self.prf = 1 / self.pri
    self.meta = pmt.dict_add(self.meta, pmt.intern("phaser:num_beams"),
                             pmt.from_long(self.num_beams))
    self.meta = pmt.dict_add(self.meta, pmt.intern("phaser:prf"),
                             pmt.from_double(self.prf))

    # Keys for metadata that can change during operation
    self.angle_key = pmt.intern("phaser:steering_angle")
    self.chan_phase_key = pmt.intern("phaser:phaser_chan_phase")
    self.chan_gain_key = pmt.intern("phaser:phaser_chan_gain")

    self.init_hardware()

    self.main_thread = gr.threading.Thread(target=self.run)
    self.main_thread.start()

  def handle_msg(self, msg):
    if pmt.is_dict(msg):
      meta, data = msg, None
    elif pmt.is_vector(msg):
      meta, data = None, msg
    elif pmt.is_pair(msg):
      meta, data = pmt.car(msg), pmt.cdr(msg)
    else:
      self.logger.warn(
          "Invalid message input: expected PMT dict, vector, or pair")
      return

    if data is not None:
      if self.radar_mode.lower() == 'pulsed':
        # Update tx data
        iq = np.asarray(pmt.c32vector_elements(data)) * 2**14
        self.sdr.tx_destroy_buffer()
        self.sdr.tx([iq, iq])
        self.started = True
      else:
        self.logger.warn(
            "Invalid message input: FMCW mode does not support arbitrary waveforms")

    # Control updates
    if meta is not None:

      if pmt.dict_has_key(meta, self.angle_key):
        angle = pmt.to_double(pmt.dict_ref(meta, self.angle_key, pmt.PMT_NIL))
        self.update_steering_angle(angle)

      if pmt.dict_has_key(meta, self.chan_phase_key):
        chan_phase = pmt.to_python(pmt.dict_ref(
            meta, self.chan_phase_key, pmt.PMT_NIL))
        for i in range(self.phaser.num_elements):
          self.phaser.set_chan_phase(i, chan_phase[i])

      if pmt.dict_has_key(meta, self.chan_gain_key):
        chan_gain = pmt.to_python(pmt.dict_ref(
            meta, self.chan_gain_key, pmt.PMT_NIL))
        for i in range(self.phaser.num_elements):
          self.phaser.set_chan_gain(i, chan_gain[i])

      self.meta = pmt.dict_update(self.meta, meta)

  def run(self):
    while not self.started:
      time.sleep(1e-3)

    while True:
      if self.stopped:
        return

      self.phaser._gpios.gpio_burst = 0
      self.phaser._gpios.gpio_burst = 1
      self.phaser._gpios.gpio_burst = 0

      data = np.asarray(self.sdr.rx())[:, self.rx_offset_samples:]

      # Extract desired range swath from each burst
      start, stop = self.burst_start_sample, self.burst_stop_sample
      if start is not None or stop is not None:
        if start is None:
          start = 0
        if stop is None:
          stop = data.shape[-1]
        data = data.reshape((self.num_beams, self.num_bursts, -1))
        data = data[:, :, start:stop]

      data = pmt.init_c32vector(data.size, data.ravel() / 2**11)
      self.message_port_pub(pmt.intern("out"), pmt.cons(self.meta, data))

  def stop(self):
    self.stopped = True
    self.main_thread.join()

    # Gracefully shut down pluto
    self.sdr.tx_destroy_buffer()
    self.sdr.rx_destroy_buffer()
    self.sdr_pins.gpio_phaser_enable = False
    return True

  def init_hardware(self):
    self.sdr = adi.ad9361(uri=self.sdr_ip)
    self.phaser = adi.CN0566(uri=self.phaser_uri, sdr=self.sdr)

    # Configure Rx
    self.sdr.sample_rate = int(self.sample_rate)
    self.sdr.rx_lo = int(self.sdr_freq)
    self.sdr.rx_enabled_channels = self.rx_enabled_channels
    self.sdr._rxadc.set_kernel_buffers_count(1)  # No stale buffers to flush
    self.sdr.gain_control_mode_chan0 = 'manual'  # manual or slow_attack
    self.sdr.gain_control_mode_chan1 = 'manual'  # manual or slow_attack
    # Between -3 and 70
    self.sdr.rx_hardwaregain_chan0 = int(self.rx_gain)
    self.sdr.rx_hardwaregain_chan1 = int(self.rx_gain)

    # Configure Tx
    self.sdr.tx_lo = int(self.sdr_freq)
    self.sdr.tx_enabled_channels = self.tx_enabled_channels
    self.sdr.tx_hardwaregain_chan0 = self.tx_hardwaregain_chan0
    self.sdr.tx_hardwaregain_chan1 = self.tx_hardwaregain_chan1
    self.sdr.tx_cyclic_buffer = self.tx_cyclic_buffer

    # Configure phaser
    self.phaser.configure(device_mode='rx')
    if self.gain_cal_file:
      self.phaser.load_gain_cal()
    if self.phase_cal_file:
      self.phaser.load_channel_cal()
    for i in range(self.phaser.num_elements):
      self.phaser.set_chan_phase(i, self.phaser_chan_phase[i])
      self.phaser.set_chan_gain(i, self.phaser_chan_gain[i])
    self.phaser._gpios.gpio_tx_sw = 0  # 0 = TX_OUT_2, 1 = TX_OUT_1
    # 1=Use onboard PLL/LO source  (0=disable PLL and VCO, and set switch to use external LO input)
    self.phaser._gpios.gpio_vctrl_1 = 1
    # 1=Send LO to transmit circuitry  (0=disable Tx path, and send LO to LO_OUT)
    self.phaser._gpios.gpio_vctrl_2 = 1

    self.sdr_pins = adi.one_bit_adc_dac(self.sdr_ip)
    # when true, each channel[1] start outputs a pulse to Pluto L10P pin (TXDATA_1V8 on Phaser schematic)
    self.sdr_pins.gpio_phaser_enable = True
    self.sdr_pins.gpio_tdd_ext_sync = True

    if self.radar_mode.lower() == 'fmcw':
      self.init_fmcw()
    else:
      self.init_pulsed()

  def init_pulsed(self) -> None:
    num_burst_samps = int(self.pri * self.sample_rate)
    num_buffer_samps = int(self.pri * self.sample_rate * self.num_bursts)
    self.sdr.rx_buffer_size = num_buffer_samps + self.rx_offset_samples
    self.phaser.frequency = int((self.phaser_output_freq + self.sdr_freq) / 4)

    # Configure TDD
    decimation = 2
    frame_length_raw = decimation*num_burst_samps - 1
    self.init_tdd(startup_delay_ms=0,
                  frame_length_raw=frame_length_raw,
                  frame_length_ms=None,
                  burst_count=0)

  def init_fmcw(self) -> None:
    # Configure ADF4159 ramping PLL
    vco_freq = self.phaser_output_freq + self.sdr_freq + self.fmcw_if_freq
    num_steps = int(self.fmcw_sweep_duration * 1e6)  # One step per us
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

    num_burst_samps = round(self.fmcw_sweep_duration * self.sample_rate)
    num_buffer_samps = num_burst_samps * self.num_bursts
    self.sdr.rx_buffer_size = num_buffer_samps + self.rx_offset_samples

    self.init_tdd(startup_delay_ms=0,
                  frame_length_raw=None,
                  frame_length_ms=self.fmcw_sweep_duration*1e3,
                  burst_count=0)

    # IF Carrier
    N = self.sdr.rx_buffer_size
    fc = int(self.fmcw_if_freq)
    ts = 1 / float(self.sdr.sample_rate)
    t = np.arange(0, N * ts, ts)
    i = np.cos(2 * np.pi * t * fc)
    q = np.sin(2 * np.pi * t * fc)
    iq = 2**14 * (i + 1j*q)

    self.sdr._ctx.set_timeout(30000)
    self.sdr._rx_init_channels()
    self.sdr.tx([iq, iq])
    self.started = True

  def init_tdd(self,
               startup_delay_ms: float,
               frame_length_raw: Optional[int] = None,
               frame_length_ms: Optional[float] = None,
               burst_count: Optional[int] = 0
               ) -> None:
    tddn = adi.tddn(self.sdr_ip)
    tddn.enable = False

    tddn.startup_delay_ms = startup_delay_ms

    if frame_length_raw is not None and frame_length_ms is None:
      tddn.frame_length_raw = frame_length_raw
    elif frame_length_ms is not None and frame_length_raw is None:
      tddn.frame_length_ms = frame_length_ms
    else:
      raise ValueError("Frame length specified in two different units")

    tddn.burst_count = burst_count
    tddn.internal_sync_period_ms = 0

    # Burst DMA SYNC
    tddn.channel[0].on_raw = 0
    tddn.channel[0].off_raw = 10
    tddn.channel[0].polarity = 0
    tddn.channel[0].enable = 1

    # RX DMA SYNC
    tddn.channel[1].on_raw = 0
    tddn.channel[1].off_raw = 10
    tddn.channel[1].polarity = 0
    tddn.channel[1].enable = 1

    # TX DMA SYNC
    tddn.channel[2].on_raw = 0
    tddn.channel[2].off_raw = 10
    tddn.channel[2].polarity = 0
    tddn.channel[2].enable = 1

    tddn.sync_external = True  # enable external sync trigger
    tddn.sync_internal = False  # enable the internal sync trigger
    tddn.sync_reset = False  # reset the internal counter when receiving a sync event
    tddn.enable = True  # enable TDD engine

  def update_steering_angle(self, angle) -> None:
    c = scipy.constants.c
    phase_delta = 2 * np.pi * self.phaser_output_freq * \
        self.phaser.element_spacing * np.sin(np.radians(angle)) / c
    self.phaser.set_beam_phase_diff(np.degrees(phase_delta))
