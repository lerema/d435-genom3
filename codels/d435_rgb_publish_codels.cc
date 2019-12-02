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
#include "acd435.h"

#include "d435_c_types.h"

#include "codels.hpp"
#include "iostream"

/* --- Task rgb_publish ------------------------------------------------- */


/** Codel d435_rgb_start of task rgb_publish.
 *
 * Triggered by d435_start.
 * Yields to d435_pause_start, d435_pub.
 */
genom_event
d435_rgb_start(bool started, const genom_context self)
{
    // Loop in start until data is initialized, then yield to poll
    if (started)
        return d435_pub;
    else
        return d435_pause_start;
}


/** Codel d435_rgb_pub of task rgb_publish.
 *
 * Triggered by d435_pub.
 * Yields to d435_pause_pub.
 */
genom_event
d435_rgb_pub(const d435_RSdata_s *data, d435_frame_s *frame,
             const d435_rgb_out *rgb_out, const genom_context self)
{
    rs2::video_frame video_data = data->rgb;
    const uint w = video_data.get_width();
    const uint h = video_data.get_height();

    if (w*h*3 <= frame->pixels._length)
    {
        frame->pixels._buffer = (uint8_t*)video_data.get_data();

        // Create timestamp
        // If using time of publishing
        // struct timeval tv;
        // gettimeofday(&tv, NULL);
        // frame->ts.sec = tv.tv_sec;
        // frame->ts.nsec = tv.tv_usec * 1000;
        // Else if using time of capture
        unsigned long ms = data->rgb.get_timestamp();
        unsigned long s = floor(ms/1000);
        unsigned long long ns = (ms - s*1000) * 1e6;
        frame->ts.sec = s;
        frame->ts.nsec = ns;
    }

    // Update port data
    *(rgb_out->data(self)) = *frame;
    rgb_out->write(self);

    return d435_pause_pub;
}


/** Codel d435_rgb_stop of task rgb_publish.
 *
 * Triggered by d435_stop.
 * Yields to d435_ether.
 */
genom_event
d435_rgb_stop(const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return d435_ether;
}


/* --- Activity set_fps ------------------------------------------------- */

/** Codel d435_set_frequency of activity set_fps.
 *
 * Triggered by d435_start.
 * Yields to d435_ether.
 */
genom_event
d435_set_frequency(uint16_t fps, const genom_context self)
{
    // TODO
    return d435_ether;
}
