#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2024 gr-phaser author.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


import time
from typing import Optional
import numpy
from gnuradio import gr
import pmt
import adi
import numpy as np


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
               burst_start_sample,
               burst_stop_sample,
               tx_gain,
               tx_cyclic_buffer,
               num_bursts,
               pri):
    gr.sync_block.__init__(self,
                           name="pluto_radar",
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
      key = f"pluto:{key}"
      self.meta = pmt.dict_add(self.meta, pmt.intern(key), pmt.to_pmt(val))

    self.started = False
    self.stopped = False
    self.logger = gr.logger(self.alias())

    # Rx params
    self.sdr_ip = uri
    self.sample_rate = sample_rate
    self.center_freq = center_freq
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
    self.tx_gain = tx_gain
    self.tx_cyclic_buffer = tx_cyclic_buffer
    # Radar params
    self.num_bursts = num_bursts
    self.pri = pri
    # Derived parameters
    self.prf = 1 / self.pri
    self.meta = pmt.dict_add(self.meta, pmt.intern("pluto:prf"),
                             pmt.from_double(self.prf))

    self.init_pluto()

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
      # Update tx data
      iq = np.asarray(pmt.c32vector_elements(data)) * 2**14
      self.sdr.tx_destroy_buffer()
      self.sdr.tx(iq)
      self.started = True

      # Control updates
    if meta is not None:
      self.meta = pmt.dict_update(self.meta, meta)

  def run(self):
    while not self.started:
      time.sleep(1e-3)

    while True:
      if self.stopped:
        return

      self.tddn.sync_soft = 1

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
    return True

  def init_pluto(self):
    self.sdr = adi.ad9361(uri=self.sdr_ip)

    # Configure Rx
    self.sdr.sample_rate = int(self.sample_rate)
    self.sdr.rx_lo = int(self.center_freq)
    self.sdr._rxadc.set_kernel_buffers_count(1)  # No stale buffers to flush
    self.sdr.rx_enabled_channels = [0]
    self.sdr.gain_control_mode_chan0 = 'manual'  # manual or slow_attack
    self.sdr.rx_hardwaregain_chan0 = int(self.rx_gain) # Between -3 and 70
    

    # Configure Tx
    self.sdr.tx_lo = int(self.center_freq)
    self.sdr.tx_enabled_channels = [0]
    self.sdr.tx_hardwaregain_chan0 = self.tx_gain
    self.sdr.tx_cyclic_buffer = self.tx_cyclic_buffer

    num_burst_samps = int(self.pri * self.sample_rate)
    num_buffer_samps = int(self.pri * self.sample_rate * self.num_bursts)
    self.sdr.rx_buffer_size = num_buffer_samps + self.rx_offset_samples

    # Configure TDD
    decimation = 2
    frame_length_raw = decimation*num_burst_samps - 1
    self.tddn = self.init_tdd(startup_delay_ms=0,
                              frame_length_raw=frame_length_raw,
                              frame_length_ms=None,
                              burst_count=0)

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

    return tddn
