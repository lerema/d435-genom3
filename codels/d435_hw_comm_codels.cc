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

#include "codels.h"
#include "fstream"
#include <iostream>


/* --- Task hw_comm ----------------------------------------------------- */


/** Codel d435_comm_start of task hw_comm.
 *
 * Triggered by d435_start.
 * Yields to d435_poll.
 */
genom_event
d435_comm_start(d435_pipe_s **pipe, bool *started,
                const genom_context self)
{
    *pipe = new d435_pipe_s();
    *started = false;
 std::cout << "hi" << std::endl;
    return d435_poll;
}


/** Codel d435_comm_poll of task hw_comm.
 *
 * Triggered by d435_poll.
 * Yields to d435_pause_poll, d435_read.
 */
genom_event
d435_comm_poll(d435_pipe_s **pipe, const genom_context self)
{
    // Loop in poll until pipe is initialized (through connect activity)
    if (!(*pipe)->init)
        return d435_pause_poll;

    // Wait for next set of frames from the camera
    (*pipe)->data = (*pipe)->pipe.wait_for_frames();

    return d435_read;
}


/** Codel d435_comm_read of task hw_comm.
 *
 * Triggered by d435_read.
 * Yields to d435_poll.
 */
genom_event
d435_comm_read(const d435_pipe_s *pipe, d435_RSdata_s **data,
               bool *started, const genom_context self)
{
    // Read RGB
    (*data)->rgb = pipe->data.get_color_frame();
    // Read RGBD
    (*data)->depth = pipe->data.get_depth_frame();
    // Update booleans
    *started = true;

    return d435_poll;
}


/** Codel d435_comm_stop of task hw_comm.
 *
 * Triggered by d435_stop.
 * Yields to d435_ether.
 */
genom_event
d435_comm_stop(const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return d435_ether;
}


/* --- Activity connect ------------------------------------------------- */

/** Codel d435_connect_start of activity connect.
 *
 * Triggered by d435_start.
 * Yields to d435_ether.
 */
genom_event
d435_connect_start(d435_ids *ids, const d435_calib *calib,
                   const d435_disto *disto, const genom_context self)
{
    // Start streaming
    rs2::pipeline_profile pipe_profile = ids->pipe->pipe.start();

    // Set configuration as written in the .json calibration file
    rs2::device dev = pipe_profile.get_device();
    rs400::advanced_mode advanced = dev.as<rs400::advanced_mode>();
    std::ifstream t("config/settings_intel.json");
    std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    advanced.load_json(preset_json);

    // Get intrinsics
    rs2::video_stream_profile stream = pipe_profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    const int h = stream.height();
    const int w = stream.width();

    rs2_intrinsics intrinsics = stream.get_intrinsics();
    float k[5] = { intrinsics.ppx,
                   intrinsics.ppy,
                   intrinsics.fx,
                   intrinsics.fy,
                   0 };
    float* d = intrinsics.coeffs;

    // Initialize intrinsics and publish
    calib->data(self)->_length = 5;
    disto->data(self)->_length = 5;
    for (int i=0; i<5; i++)
    {
        calib->data(self)->_buffer[i] = k[i];
        disto->data(self)->_buffer[i] = d[i];
    }
    calib->write(self);
    disto->write(self);

    // Initialize sequence for frame
    ids->frame.height = h;
    ids->frame.width = w;

    // ids->frame.pixels._maximum = h*w*3;
    if (genom_sequence_reserve(&(ids->frame.pixels), h*w*3)  == -1) {
        d435_mem_error_detail d;
        snprintf(d.what, sizeof(d.what), "unable to allocate rgb frame memory");
    }

    // Initialize sequence for point cloud
    ids->pc.isRegistered = false;
    if (genom_sequence_reserve(&(ids->pc.points), 0)  == -1) {
        d435_mem_error_detail d;
        snprintf(d.what, sizeof(d.what), "unable to allocate 3d point memory");
    }
    if (genom_sequence_reserve(&(ids->pc.colors), 0)  == -1) {
        d435_mem_error_detail d;
        snprintf(d.what, sizeof(d.what), "unable to allocate points color memory");
    }


    // Init boolean
    ids->pipe->init = true;

    return d435_ether;
}
