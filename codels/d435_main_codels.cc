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

#include <err.h>
#include <cmath>

/* --- Task main -------------------------------------------------------- */


/** Codel d435_main_start of task main.
 *
 * Triggered by d435_start.
 * Yields to d435_poll.
 */
genom_event
d435_main_start(d435_ids *ids, const d435_extrinsics *extrinsics,
                const d435_frame *frame, const genom_context self)
{
    *extrinsics->data(self) = {0,0,0,0,0,0};
    extrinsics->write(self);

    ids->pipe = new or_camera_pipe();
    ids->info.started = false;
    ids->info.frequency = 30;
    snprintf(ids->info.format, sizeof(ids->info.format), "RGB8");
    ids->info.size = {1280, 720};
    ids->info.compression_rate = -1;

    frame->open("raw", self);
    frame->open("compressed", self);

    genom_sequence_reserve(&(frame->data("raw", self)->pixels), 0);
    genom_sequence_reserve(&(frame->data("compressed", self)->pixels), 0);

    return d435_poll;
}


/** Codel d435_main_poll of task main.
 *
 * Triggered by d435_poll.
 * Yields to d435_pause_poll, d435_pub.
 */
genom_event
d435_main_poll(bool started, or_camera_pipe **pipe,
               const genom_context self)
{
    // Loop in poll until pipe is initialized (through connect activity)
    if (!started)
        return d435_pause_poll;

    // Wait for next set of frames from the camera
    try {
        (*pipe)->data = (*pipe)->pipe.wait_for_frames(15000);
    }
    catch (rs2::error e) {
        warnx("%s\n", e.what());
        return d435_pause_poll;
    }

    return d435_pub;
}


/** Codel d435_main_pub of task main.
 *
 * Triggered by d435_pub.
 * Yields to d435_poll.
 */
genom_event
d435_main_pub(int16_t compression_rate, const or_camera_pipe *pipe,
              const d435_frame *frame, const genom_context self)
{
    or_sensor_frame* rfdata = frame->data("raw", self);
    or_sensor_frame* cfdata = frame->data("compressed", self);

    video_frame rsframe = pipe->data.get_color_frame();
    const uint16_t w = rsframe.get_width();
    const uint16_t h = rsframe.get_height();
    const uint16_t c = rsframe.get_bytes_per_pixel();
    const double ms = rsframe.get_timestamp();

    if (h*w*c != rfdata->pixels._maximum)
    {
        if (genom_sequence_reserve(&(rfdata->pixels), h*w*c)  == -1) {
            d435_e_mem_detail d;
            snprintf(d.what, sizeof(d.what), "unable to allocate frame memory");
            warnx("%s", d.what);
            return d435_e_mem(&d,self);
        }
        rfdata->pixels._length = h*w*c;
        rfdata->height = h;
        rfdata->width = w;
        rfdata->bpp = c;
        rfdata->compressed = false;
    }

    rfdata->pixels._buffer = (uint8_t*) rsframe.get_data();
    rfdata->ts.sec = floor(ms/1000);
    rfdata->ts.nsec = (ms - (double)rfdata->ts.sec*1000) * 1e6;

    if (compression_rate != -1)
    {
        int16_t type;
        if      (rfdata->bpp == 2) type = CV_16UC1;
        else if (rfdata->bpp == 3) type = CV_8UC3;
        else if (rfdata->bpp == 4) type = CV_8UC4;

        Mat cvframe = Mat(
            Size(rfdata->width, rfdata->height),
            type,
            rfdata->pixels._buffer,
            Mat::AUTO_STEP
        );

        // JPEG does not handle 16bits data and alpha channels
        int16_t cc = 3;    // compressed image channels
        if (rfdata->bpp == 2) {
            cvframe.convertTo(cvframe, CV_8U, 1/256.0);
            cc = 1;
        }
        else if (rfdata->bpp == 4) {
            cvtColor(cvframe, cvframe, CV_RGBA2RGB);
            cc = 3;
        }

        std::vector<int32_t> compression_params;
        compression_params.push_back(IMWRITE_JPEG_QUALITY);
        compression_params.push_back(compression_rate);

        std::vector<uint8_t> buf;
        imencode(".jpg", cvframe, buf, compression_params);

        if (buf.size() > cfdata->pixels._maximum)
            if (genom_sequence_reserve(&(cfdata->pixels), buf.size())  == -1) {
                d435_e_mem_detail d;
                snprintf(d.what, sizeof(d.what), "unable to allocate frame memory");
                warnx("%s", d.what);
                return d435_e_mem(&d,self);
            }
        cfdata->pixels._length = buf.size();
        cfdata->height = rfdata->height;
        cfdata->width = rfdata->width;
        cfdata->bpp = cc;
        cfdata->compressed = true;

        cfdata->pixels._buffer = buf.data();
        cfdata->ts = rfdata->ts;
    }

    frame->write("raw", self);
    frame->write("compressed", self);

    return d435_poll;
}


/* --- Activity connect ------------------------------------------------- */

/** Codel d435_connect of activity connect.
 *
 * Triggered by d435_start.
 * Yields to d435_ether.
 * Throws d435_e_rs, d435_e_io.
 */
genom_event
d435_connect(const char serial[12], or_camera_pipe **pipe,
             const or_camera_info *info, bool *started,
             const d435_intrinsics *intrinsics,
             const genom_context self)
{
    if (*started)
    {
        d435_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "already connected to device, disconnect() first");
        warnx("%s", d.what);
        return d435_e_io(&d,self);
    }
    else
    {
        rs2_intrinsics intrinsics_rs2;

        try {
            // Start streaming
            config cfg;

            rs2_format fmt = RS2_FORMAT_RGB8;
            if (!strcmp(info->format,"RGBA8")) fmt = RS2_FORMAT_RGBA8;
            if (!strcmp(info->format,"Y16")) fmt = RS2_FORMAT_Y16;

            cfg.enable_stream(RS2_STREAM_COLOR, info->size.w, info->size.h, fmt, info->frequency);

            pipeline_profile pipe_profile = (*pipe)->pipe.start(cfg);
            video_stream_profile stream = pipe_profile.get_stream(RS2_STREAM_COLOR).as<video_stream_profile>();
            intrinsics_rs2 = stream.get_intrinsics();
        } catch (rs2::error& e) {
            d435_e_rs_detail d;
            snprintf(d.what, sizeof(d.what), "%s", e.what());
            warnx("rs error: %s", d.what);
            return d435_e_rs(&d,self);
        }

        // Publish intrinsincs
        or_sensor_intrinsics* intr_data = intrinsics->data(self);
        *intr_data = {
            .calib = {
                intrinsics_rs2.fx,
                intrinsics_rs2.fy,
                intrinsics_rs2.ppx,
                intrinsics_rs2.ppy,
                0
            },
            .disto = {
                intrinsics_rs2.coeffs[0],
                intrinsics_rs2.coeffs[1],
                intrinsics_rs2.coeffs[2],
                intrinsics_rs2.coeffs[3],
                intrinsics_rs2.coeffs[4]
            }
        };
        intrinsics->write(self);

        // Init boolean
        *started = true;
    }

    warnx("connected to D435 device");
    return d435_ether;
}


/* --- Activity disconnect ---------------------------------------------- */

/** Codel d435_disconnect of activity disconnect.
 *
 * Triggered by d435_start.
 * Yields to d435_ether.
 * Throws d435_e_rs.
 */
genom_event
d435_disconnect(or_camera_pipe **pipe, bool *started,
                const genom_context self)
{
    try {
        (*pipe)->pipe.stop();
    } catch (rs2::error& e) {
        d435_e_rs_detail d;
        snprintf(d.what, sizeof(d.what), "%s", e.what());
        warnx("rs error: %s", d.what);
        return d435_e_rs(&d,self);
    }
    *started = false;

    warnx("disconnected from device");
    return d435_ether;
}


/* --- Activity set_extrinsics ------------------------------------------ */

/** Codel d435_set_extrinsics of activity set_extrinsics.
 *
 * Triggered by d435_start.
 * Yields to d435_ether.
 * Throws d435_e_rs.
 */
genom_event
d435_set_extrinsics(const sequence6_float *ext_values,
                    const d435_extrinsics *extrinsics,
                    const genom_context self)
{
    *extrinsics->data(self) = {
        ext_values->_buffer[0],
        ext_values->_buffer[1],
        ext_values->_buffer[2],
        ext_values->_buffer[3],
        ext_values->_buffer[4],
        ext_values->_buffer[5],
    };
    extrinsics->write(self);

    warnx("new extrinsic calibration");
    return d435_ether;
}
