/*
 * Copyright 2024 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(sum_beams.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(ae0b39e78da6848bd7cc0fc647a99911)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/phaser/sum_beams.h>
// pydoc.h is automatically generated in the build directory
#include <sum_beams_pydoc.h>

void bind_sum_beams(py::module& m)
{

    using sum_beams    = gr::phaser::sum_beams;


    py::class_<sum_beams, gr::block, gr::basic_block,
        std::shared_ptr<sum_beams>>(m, "sum_beams", D(sum_beams))

        .def(py::init(&sum_beams::make),
           D(sum_beams,make)
        )
        



        ;




}








