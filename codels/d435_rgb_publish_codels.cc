/*
 * Copyright (c) 1996-2015 CNRS/LAAS
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include "acd435.h"

#include "d435_c_types.h"

#include "codels.h"
// #include "opencv2/opencv.hpp"

/* --- Task rgb_publish ------------------------------------------------- */


/** Codel d435_rgb_start of task rgb_publish.
 *
 * Triggered by d435_start.
 * Yields to d435_pub.
 */
genom_event
d435_rgb_start(const d435_RSdata_s *data, const genom_context self)
{
    // Loop in start until data is initialized, then yield to poll
    if (data->init)
        return d435_pub;
    else
        return d435_start;
}


/** Codel d435_rgb_pub of task rgb_publish.
 *
 * Triggered by d435_pub.
 * Yields to d435_pause_pub.
 */
genom_event
d435_rgb_pub(const d435_RSdata_s *data, d435_frame_s *frame,
             const d435_curr_frame *curr_frame,
             const genom_context self)
{
    const uint w = data->rgb.get_width();
    const uint h = data->rgb.get_height();

    if (w*h*3 <= frame->pixels._length)
    {

        // cv::Mat cvFrame(cv::Size(w, h), CV_8UC3, (void*)data->rgb.get_data(), cv::Mat::AUTO_STEP);
        // cv::cvtColor(cvFrame, cvFrame, cv::COLOR_BGR2RGB);
        // frame->pixels._buffer = cvFrame.data;
        frame->pixels._buffer = (uint8_t*)data->rgb.get_data();

        // Create timestamp
        // If using time of publishing
        // struct timeval tv;
        // gettimeofday(&tv, NULL);
        // frame->timeStamp.sec = tv.tv_sec;
        // frame->timeStamp.nsec = tv.tv_usec * 1000;

        // Else if using time of capture
        unsigned long ms = data->rgb.get_timestamp();
        unsigned long s = floor(ms/1000);
        unsigned long long ns = (ms - s*1000) * 1e6;
        frame->timeStamp.sec = s;
        frame->timeStamp.nsec = ns;
    }

    // Update port data
    *(curr_frame->data(self)) = *frame;
    curr_frame->write(self);

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
