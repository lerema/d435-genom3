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
                const genom_context self)
{
    extrinsics->write(self);
    ids->pipe = new or_camera_pipe();
    ids->info.started = false;
    ids->info.frequency = 30;
    snprintf(ids->info.format, sizeof(ids->info.format), "RGB8");
    ids->info.size = {1280, 720};
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
    catch (error e) {
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
d435_main_pub(const or_camera_pipe *pipe, const d435_frame *frame,
              const genom_context self)
{
    or_sensor_frame* fdata = frame->data(self);

    video_frame rsframe = pipe->data.get_color_frame();
    const uint16_t w = rsframe.get_width();
    const uint16_t h = rsframe.get_height();
    const uint16_t c = rsframe.get_bytes_per_pixel();

    if (h*w*c != fdata->pixels._maximum)
    {
        if (genom_sequence_reserve(&(fdata->pixels), h*w*c)  == -1) {
            d435_e_mem_detail d;
            snprintf(d.what, sizeof(d.what), "unable to allocate frame memory");
            warnx("%s", d.what);
            return d435_e_mem(&d,self);
        }
        fdata->pixels._length = h*w*c;
        fdata->height = h;
        fdata->width = w;
        fdata->bpp = c;
    }

    fdata->pixels._buffer = (uint8_t*) rsframe.get_data();

    double ms = rsframe.get_timestamp();
    fdata->ts.sec = floor(ms/1000);
    fdata->ts.nsec = (ms - (double)fdata->ts.sec*1000) * 1e6;

    frame->write(self);

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
d435_connect(or_camera_pipe **pipe, const or_camera_info *info,
             bool *started, const d435_intrinsics *intrinsics,
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
        } catch (error& e) {
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
    } catch (error& e) {
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
