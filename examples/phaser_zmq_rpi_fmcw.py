#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
# GNU Radio version: v3.10.11.0-1-gee27d6f3

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import zeromq
from gnuradio.phaser import phaser_radar
import numpy as np
import threading



class phaser_zmq_rpi_fmcw(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Not titled yet", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Not titled yet")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "phaser_zmq_rpi_fmcw")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 5e6
        self.angle = angle = 0

        ##################################################
        # Blocks
        ##################################################

        self.zeromq_pub_msg_sink_0 = zeromq.pub_msg_sink('tcp://169.254.227.151:3001', 100, True)
        self.phaser_phaser_radar_0 = phaser_radar.blk(
          'ip:pluto.local',
          samp_rate,
          2.1e9,
          [0, 1],
          20,
          0,
          0,
          (-1),
          [0, 1],
          0,
          0,
          True,
          'ip:phaser.local',
          10e9,
          [0, 0, 0, 0, 0, 0, 0, 0],
          [127]*8,
          '',
          '',
          'fmcw',
          5,
          1/10e3,
          500e-6,
          100e3,
          500e6,
          'single_sawtooth_burst',
        )


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.phaser_phaser_radar_0, 'out'), (self.zeromq_pub_msg_sink_0, 'in'))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "phaser_zmq_rpi_fmcw")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate

    def get_angle(self):
        return self.angle

    def set_angle(self, angle):
        self.angle = angle




def main(top_block_cls=phaser_zmq_rpi_fmcw, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()
    tb.flowgraph_started.set()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
