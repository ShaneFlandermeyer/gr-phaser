/* -*- c++ -*- */
/*
 * Copyright 2024 gr-phaser author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "sum_beams_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace phaser {

sum_beams::sptr sum_beams::make(std::string beam_key_pattern)
{
    return gnuradio::make_block_sptr<sum_beams_impl>(beam_key_pattern);
}


/*
 * The private constructor
 */
sum_beams_impl::sum_beams_impl(std::string beam_key_pattern)
    : gr::block(
          "sum_beams", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)),
      beam_key_pattern(beam_key_pattern),
      beam_key_regex(std::regex(beam_key_pattern))
{
    message_port_register_in(pmt::mp("in"));
    message_port_register_out(pmt::mp("out"));
    set_msg_handler(pmt::mp("in"), [this](pmt::pmt_t msg) { this->handle_msg(msg); });
}

/*
 * Our virtual destructor.
 */
sum_beams_impl::~sum_beams_impl() {}

void sum_beams_impl::handle_msg(pmt::pmt_t msg)
{
    // Extract beam data from the message
    pmt::pmt_t meta = msg;
    pmt::pmt_t items = pmt::dict_items(meta);
    pmt::pmt_t item, key, value;
    std::vector<std::vector<gr_complex>> beams;
    for (size_t i = 0; i < pmt::length(items); i++) {
        item = pmt::nth(i, items);
        key = pmt::car(item);
        value = pmt::cdr(item);
        if (std::regex_match(pmt::symbol_to_string(key), beam_key_regex)) {
            beams.push_back(pmt::c32vector_elements(value));
            meta = pmt::dict_delete(meta, key);
        }
    }

    // Sum beams to produce a single data vector
    std::vector<gr_complex> sum_beam(beams[0].size(), 0);
    for (auto beam : beams) {
        for (size_t i = 0; i < beam.size(); i++) {
            sum_beam[i] += beam[i];
        }
    }

    // Send data to the output port as a PDU
    pmt::pmt_t data = pmt::init_c32vector(sum_beam.size(), sum_beam.data());
    message_port_pub(pmt::mp("out"), pmt::cons(meta, data));
}

} /* namespace phaser */
} /* namespace gr */
