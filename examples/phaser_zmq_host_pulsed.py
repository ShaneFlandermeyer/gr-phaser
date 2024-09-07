#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
# GNU Radio version: 3.10.9.2

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import Qt
from gnuradio import plasma
import sip
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import phaser
from gnuradio import zeromq
import numpy as np



class phaser_zmq_host_pulsed(gr.top_block, Qt.QWidget):

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

        self.settings = Qt.QSettings("GNU Radio", "phaser_zmq_host_pulsed")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 30e6
        self.rpi_ip = rpi_ip = "169.254.227.151"
        self.ncpi = ncpi = 64
        self.center_freq = center_freq = 10e9
        self.bandwidth = bandwidth = (2/3)*samp_rate

        ##################################################
        # Blocks
        ##################################################

        self.zeromq_sub_msg_source_0 = zeromq.sub_msg_source(f"tcp://{rpi_ip}:3001", 100, False)
        self.zeromq_pub_msg_sink_0 = zeromq.pub_msg_sink(f"tcp://{rpi_ip}:2001", 100, False)
        self.qtgui_time_sink_x_0 = qtgui.time_sink_c(
            1024, #size
            samp_rate, #samp_rate
            "", #name
            0, #number of inputs
            None # parent
        )
        self.qtgui_time_sink_x_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0.enable_tags(True)
        self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_0.enable_autoscale(False)
        self.qtgui_time_sink_x_0.enable_grid(False)
        self.qtgui_time_sink_x_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_0.enable_control_panel(False)
        self.qtgui_time_sink_x_0.enable_stem_plot(False)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(2):
            if len(labels[i]) == 0:
                if (i % 2 == 0):
                    self.qtgui_time_sink_x_0.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.qtgui_time_sink_x_0.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.qtgui_time_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_time_sink_x_0_win)
        self.plasma_range_doppler_sink_0 = plasma.range_doppler_sink(samp_rate, ncpi, center_freq)
        self.plasma_range_doppler_sink_0.set_metadata_keys('core:sample_rate', 'n_matrix_col', 'phaser:frequency', 'dynamic_range', 'phaser:prf', 'radar:duration', 'detection_indices')
        self.plasma_range_doppler_sink_0.set_dynamic_range(60)
        self.plasma_range_doppler_sink_0.set_msg_queue_depth(1)
        self._plasma_range_doppler_sink_0_win = sip.wrapinstance(self.plasma_range_doppler_sink_0.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._plasma_range_doppler_sink_0_win)
        self.plasma_pulse_doppler_0 = plasma.pulse_doppler(ncpi, ncpi)
        self.plasma_pulse_doppler_0.set_msg_queue_depth(1)
        self.plasma_pulse_doppler_0.set_backend(plasma.Device.DEFAULT)
        self.plasma_pulse_doppler_0.init_meta_dict('doppler_fft_size')
        self.plasma_lfm_source_0 = plasma.lfm_source(bandwidth, -bandwidth/2, 20e-6, samp_rate, 0)
        self.plasma_lfm_source_0.init_meta_dict('radar:bandwidth', 'radar:start_freq', 'radar:duration', 'core:sample_rate', 'core:label', 'radar:prf')
        self.phaser_sum_beams_0 = phaser.sum_beams('phaser:num_beams')


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.phaser_sum_beams_0, 'out'), (self.plasma_pulse_doppler_0, 'rx'))
        self.msg_connect((self.phaser_sum_beams_0, 'out'), (self.qtgui_time_sink_x_0, 'in'))
        self.msg_connect((self.plasma_lfm_source_0, 'out'), (self.plasma_pulse_doppler_0, 'tx'))
        self.msg_connect((self.plasma_lfm_source_0, 'out'), (self.zeromq_pub_msg_sink_0, 'in'))
        self.msg_connect((self.plasma_pulse_doppler_0, 'out'), (self.plasma_range_doppler_sink_0, 'in'))
        self.msg_connect((self.zeromq_sub_msg_source_0, 'out'), (self.phaser_sum_beams_0, 'in'))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "phaser_zmq_host_pulsed")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_bandwidth((2/3)*self.samp_rate)
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate)

    def get_rpi_ip(self):
        return self.rpi_ip

    def set_rpi_ip(self, rpi_ip):
        self.rpi_ip = rpi_ip

    def get_ncpi(self):
        return self.ncpi

    def set_ncpi(self, ncpi):
        self.ncpi = ncpi

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq

    def get_bandwidth(self):
        return self.bandwidth

    def set_bandwidth(self, bandwidth):
        self.bandwidth = bandwidth




def main(top_block_cls=phaser_zmq_host_pulsed, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()

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
