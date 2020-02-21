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
#include "opencv2/opencv.hpp"

using namespace cv;

/* --- Global variables ------------------------------------------------- */
bool display = false;
bool record = false;
VideoWriter recorder_rgb;
VideoWriter recorder_ir;

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
 * Yields to d435_visu.
 */
genom_event
d435_rgb_pub(const d435_RSdata_s *data, d435_frame_s *rgb,
             d435_frame_s *ir, const d435_rgb_out *rgb_out,
             const genom_context self)
{
    const uint16_t w = ((rs2::video_frame) data->rgb).get_width();
    const uint16_t h = ((rs2::video_frame) data->rgb).get_height();

    if (w*h*3 <= rgb->pixels._length)
    {
        rgb->pixels._buffer = (uint8_t*) data->rgb.get_data();
        ir->pixels._buffer = (uint8_t*) data->ir.get_data();

        // Create timestamp
        uint32_t ms = data->rgb.get_timestamp();
        uint32_t s = floor(ms/1000);
        uint64_t ns = (ms - s*1000) * 1e6;
        rgb->ts.sec = s;
        rgb->ts.nsec = ns;
    }

    // Update port data
    *(rgb_out->data(self)) = *rgb;
    rgb_out->write(self);

    return d435_visu;
}


/** Codel d435_rgb_visu of task rgb_publish.
 *
 * Triggered by d435_visu.
 * Yields to d435_pause_start.
 */
genom_event
d435_rgb_visu(const d435_frame_s *rgb, const d435_frame_s *ir,
              const genom_context self)
{
    Mat cvRGB(Size(rgb->width, rgb->height), CV_8UC3, (void*)rgb->pixels._buffer, Mat::AUTO_STEP);
    Mat cvBGR;
    cvtColor(cvRGB, cvBGR, COLOR_RGB2BGR);  // realsense data is RGB
    Mat cvIR(Size(ir->width, ir->height), CV_8UC1, (void*)ir->pixels._buffer, Mat::AUTO_STEP);
    equalizeHist(cvIR, cvIR);
    applyColorMap(cvIR, cvIR, COLORMAP_JET);

    if (display)
    {
        imshow("D435 RGB", cvBGR);
        imshow("D435 IR", cvIR);
        waitKey(1);
    }
    if (record)
    {
        recorder_rgb.write(cvBGR);
        recorder_ir.write(cvIR);
    }

    return d435_pause_start;
}


/* --- Activity display ------------------------------------------------- */

/** Codel d435_disp_start of activity display.
 *
 * Triggered by d435_start.
 * Yields to d435_ether.
 */
genom_event
d435_disp_start(const genom_context self)
{
    display = true;
    return d435_ether;
}


/* --- Activity stop_display -------------------------------------------- */

/** Codel d435_disp_stop of activity stop_display.
 *
 * Triggered by d435_start.
 * Yields to d435_ether.
 */
genom_event
d435_disp_stop(const genom_context self)
{
    display = false;
    destroyAllWindows();
    return d435_ether;
}


/* --- Activity record -------------------------------------------------- */

/** Codel d435_rec_start of activity record.
 *
 * Triggered by d435_start.
 * Yields to d435_ether.
 */
genom_event
d435_rec_start(const char path[64], const d435_frame_s *rgb,
               const d435_frame_s *ir, const genom_context self)
{
    char path_rgb[64];
    snprintf(path_rgb, sizeof(path_rgb), "%s/rgb.avi", path);
    recorder_rgb = VideoWriter(path_rgb,CV_FOURCC('M','J','P','G'), 60, Size(rgb->width,rgb->height));
    char path_ir[64];
    snprintf(path_ir, sizeof(path_ir), "%s/ir.avi", path);
    recorder_ir = VideoWriter(path_ir,CV_FOURCC('M','J','P','G'), 60, Size(ir->width,ir->height));
    record = true;
    return d435_ether;
}


/* --- Activity stop_record --------------------------------------------- */

/** Codel d435_rec_stop of activity stop_record.
 *
 * Triggered by d435_start.
 * Yields to d435_ether.
 */
genom_event
d435_rec_stop(const genom_context self)
{
    recorder_rgb.release();
    recorder_ir.release();
    record = false;
    return d435_ether;
}
