/* -*- c++ -*- */
/*
 * Copyright 2024 gr-phaser author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_PHASER_SUM_BEAMS_H
#define INCLUDED_PHASER_SUM_BEAMS_H

#include <gnuradio/block.h>
#include <gnuradio/phaser/api.h>

namespace gr {
namespace phaser {

/*!
 * \brief <+description of block+>
 * \ingroup phaser
 *
 */
class PHASER_API sum_beams : virtual public gr::block
{
public:
    typedef std::shared_ptr<sum_beams> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of phaser::sum_beams.
     *
     * To avoid accidental use of raw pointers, phaser::sum_beams's
     * constructor is in a private implementation
     * class. phaser::sum_beams::make is the public interface for
     * creating new instances.
     */
    static sptr make(std::string num_beams_key);
};

} // namespace phaser
} // namespace gr

#endif /* INCLUDED_PHASER_SUM_BEAMS_H */
