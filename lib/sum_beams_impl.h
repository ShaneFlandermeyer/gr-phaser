/* -*- c++ -*- */
/*
 * Copyright 2024 gr-phaser author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_PHASER_SUM_BEAMS_IMPL_H
#define INCLUDED_PHASER_SUM_BEAMS_IMPL_H

#include <gnuradio/phaser/sum_beams.h>

namespace gr {
namespace phaser {

class sum_beams_impl : public sum_beams
{
private:
    pmt::pmt_t num_beams_key;
    size_t num_beams;

    void handle_msg(pmt::pmt_t msg);

public:
    sum_beams_impl(std::string num_beams_key);
    ~sum_beams_impl();
};

} // namespace phaser
} // namespace gr

#endif /* INCLUDED_PHASER_SUM_BEAMS_IMPL_H */
