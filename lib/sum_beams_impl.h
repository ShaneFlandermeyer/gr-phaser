/* -*- c++ -*- */
/*
 * Copyright 2024 gr-phaser author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_PHASER_SUM_BEAMS_IMPL_H
#define INCLUDED_PHASER_SUM_BEAMS_IMPL_H

#include <gnuradio/phaser/sum_beams.h>
#include <regex>

namespace gr {
namespace phaser {

class sum_beams_impl : public sum_beams
{
private:
    std::string beam_key_pattern;
    std::regex beam_key_regex;


    void handle_msg(pmt::pmt_t msg);

public:
    sum_beams_impl(std::string beam_key_pattern);
    ~sum_beams_impl();
};

} // namespace phaser
} // namespace gr

#endif /* INCLUDED_PHASER_SUM_BEAMS_IMPL_H */
