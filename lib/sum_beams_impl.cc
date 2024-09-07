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

sum_beams::sptr sum_beams::make(std::string num_beams_key)
{
    return gnuradio::make_block_sptr<sum_beams_impl>(num_beams_key);
}


/*
 * The private constructor
 */
sum_beams_impl::sum_beams_impl(std::string num_beams_key)
    : gr::block(
          "sum_beams", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)),
      num_beams_key(pmt::string_to_symbol(num_beams_key))
{
    message_port_register_in(pmt::mp("in"));
    message_port_register_out(pmt::mp("out"));
    set_msg_handler(pmt::mp("in"), [this](pmt::pmt_t msg) { handle_msg(msg); });
}

void sum_beams_impl::handle_msg(pmt::pmt_t msg)
{
    pmt::pmt_t meta, data;
    if (pmt::is_dict(msg)) {
        meta = msg;
        data = pmt::PMT_NIL;
    } else if (pmt::is_vector(msg)) {
        meta = pmt::PMT_NIL;
        data = msg;
    } else if (pmt::is_pair(msg)) {
        meta = pmt::car(msg);
        data = pmt::cdr(msg);
    }

    if (not pmt::is_null(meta)) {
        if (pmt::dict_has_key(meta, num_beams_key)) {
            num_beams = pmt::to_uint64(pmt::dict_ref(meta, num_beams_key, pmt::PMT_NIL));
        }
    }

    if (not pmt::is_null(data)) {
        // Sum beams from the input
        // Assumes beams are stacked sequentially in the data vector
        // i.e. [beam1, beam2, beam3, ..., beamN]
        size_t total_samps = pmt::length(data);
\
        std::vector<gr_complex> sum_beam(total_samps / num_beams, 0);
        const gr_complex* data_ptr = pmt::c32vector_elements(data, total_samps);
        for (size_t i = 0; i < total_samps; i++) {
            sum_beam[i % sum_beam.size()] += data_ptr[i];
        }
        data = pmt::init_c32vector(sum_beam.size(), sum_beam.data());
    }

    // Do something with the message
    message_port_pub(pmt::mp("out"), pmt::cons(meta, data));
}

/*
 * Our virtual destructor.
 */
sum_beams_impl::~sum_beams_impl() {}

} /* namespace phaser */
} /* namespace gr */
