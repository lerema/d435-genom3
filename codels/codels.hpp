/*
 * Copyright (c) 2019 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                              Martin Jacquet - November 2019
 */
#ifndef H_D435_CODELS
#define H_D435_CODELS

#include "d435_c_types.h"

#include "fstream"
#include "iostream"

#include "librealsense2/rs.hpp"
#include "librealsense2/rs_advanced_mode.hpp"

struct d435_pipe_s{
    bool                init = false;
    rs2::pipeline       pipe;
    rs2::frameset       data;
};

struct d435_RSdata_s{
    rs2::frame          rgb;
    rs2::frame          ir;
    rs2::frame          pc;

    d435_RSdata_s(rs2::frameset data) {
        rgb = data.get_color_frame();
        ir = data.get_infrared_frame();
        pc = data.get_depth_frame();
    };
};

#endif /* H_D435_CODELS */
