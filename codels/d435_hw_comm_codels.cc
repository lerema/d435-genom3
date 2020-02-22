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

/* --- Task hw_comm ----------------------------------------------------- */


/** Codel d435_comm_start of task hw_comm.
 *
 * Triggered by d435_start.
 * Yields to d435_poll.
 * Throws d435_e_rs.
 */
genom_event
d435_comm_start(d435_ids *ids, const d435_extrinsics *extrinsics,
                const genom_context self)
{
    *extrinsics->data(self) = {0,0,0, 0,0,0};
    extrinsics->write(self);
    ids->pipe = new d435_pipe_s();
    ids->started = false;
    return d435_poll;
}


/** Codel d435_comm_poll of task hw_comm.
 *
 * Triggered by d435_poll.
 * Yields to d435_pause_poll, d435_read.
 * Throws d435_e_rs.
 */
genom_event
d435_comm_poll(d435_pipe_s **pipe, const genom_context self)
{
    // Loop in poll until pipe is initialized (through connect activity)
    if (!(*pipe)->init)
        return d435_pause_poll;

    // Wait for next set of frames from the camera
    try {
        (*pipe)->data = (*pipe)->pipe.wait_for_frames();
    }
    catch (rs2::error e) {
        d435_e_rs_detail d;
        snprintf(d.what, sizeof(d.what), "%s", e.what());
        printf("d435: %s\n", d.what);
        return d435_e_rs(&d,self);
    }

    return d435_read;
}


/** Codel d435_comm_read of task hw_comm.
 *
 * Triggered by d435_read.
 * Yields to d435_poll.
 * Throws d435_e_rs.
 */
genom_event
d435_comm_read(const d435_pipe_s *pipe, d435_RSdata_s **data,
               bool *started, const genom_context self)
{
    // Read
    (*data)->rgb = pipe->data.get_color_frame();
    (*data)->ir = pipe->data.get_infrared_frame();
    (*data)->pc = pipe->data.get_depth_frame();

    // Update booleans
    *started = true;

    return d435_poll;
}



/* --- Activity connect ------------------------------------------------- */

/** Codel d435_connect of activity connect.
 *
 * Triggered by d435_start.
 * Yields to d435_ether.
 * Throws d435_e_rs, d435_e_mem, d435_e_d435.
 */
genom_event
d435_connect(d435_ids *ids, const d435_intrinsics *intrinsics,
             const genom_context self)
{
    std::cout << "d435: initializing connection to hardware..." << std::endl;

    // Test if device already connected
    if (ids->pipe->init) {
        d435_e_d435_detail d;
        snprintf(d.what, sizeof(d.what), "device already connected");
        printf("d435: %s\n", d.what);
        return d435_e_d435(&d,self);
    }
    if (!ids->data)
        {
        rs2::video_stream_profile* stream, * streamIR;
        try
        {
            // Start streaming
            rs2::config cfg;
            cfg.enable_all_streams();
            cfg.disable_stream(RS2_STREAM_ACCEL);
            cfg.disable_stream(RS2_STREAM_GYRO);
            cfg.disable_stream(RS2_STREAM_FISHEYE, 0);
            cfg.disable_stream(RS2_STREAM_FISHEYE, 1);
            cfg.disable_stream(RS2_STREAM_GPIO);
            cfg.disable_stream(RS2_STREAM_POSE);
            cfg.disable_stream(RS2_STREAM_CONFIDENCE);
            rs2::pipeline_profile pipe_profile = ids->pipe->pipe.start(cfg);

            // Set configuration as written in the .json calibration file
            rs2::device dev = pipe_profile.get_device();
            rs400::advanced_mode advanced = dev.as<rs400::advanced_mode>();

            // Load config file
            const char* config_path = std::getenv("GENOM_DEVEL");
            if (config_path == NULL) {
                d435_e_d435_detail d;
                snprintf(d.what, sizeof(d.what), "$GENOM_DEVEL env variable not found");
                printf("d435: %s ; it is required for loading the configuration file\n", d.what);
                return d435_e_d435(&d,self);
            }
            char config_file[128];
            strcpy(config_file, config_path); // copy string one into the result.
            strcat(config_file, "/etc/d435.json");
            std::ifstream config(config_file);
            if (config.is_open())
                printf("d435: configuration loaded\n");
            else {
                d435_e_d435_detail d;
                snprintf(d.what, sizeof(d.what), "configuration file not found");
                printf("d435: %s\nd435: is d435 installed in $GENOM_DEVEL? check $GENOM_DEVEL/etc folder\n", d.what);
                return d435_e_d435(&d,self);
            }
            std::string preset_json((std::istreambuf_iterator<char>(config)), std::istreambuf_iterator<char>());
            advanced.load_json(preset_json);

            // Get intrinsics
            stream = new rs2::video_stream_profile(pipe_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>());
            streamIR = new rs2::video_stream_profile(pipe_profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>());
        }
        catch (rs2::error e)
        {
            d435_e_rs_detail d;
            snprintf(d.what, sizeof(d.what), "%s", e.what());
            printf("d435: %s\n", d.what);
            return d435_e_rs(&d,self);
        }

        rs2_intrinsics intrinsics_rs2 = stream->get_intrinsics();
        float k[5] = { intrinsics_rs2.fx,
                       intrinsics_rs2.fy,
                       intrinsics_rs2.ppx,
                       intrinsics_rs2.ppy,
                       0 };
        float* d = intrinsics_rs2.coeffs;

        intrinsics->data(self)->calib._length = 5;
        intrinsics->data(self)->disto._length = 5;
        for (int i=0; i<5; i++) {
            intrinsics->data(self)->calib._buffer[i] = k[i];
            intrinsics->data(self)->disto._buffer[i] = d[i];
        }
        intrinsics->write(self);

        // Initialize sequence for color frame
        ids->rgb.height = stream->height();
        ids->rgb.width = stream->width();
        ids->rgb.bpp = 3;
        ids->rgb.pixels._length = ids->rgb.height*ids->rgb.width*ids->rgb.bpp;
        if (genom_sequence_reserve(&(ids->rgb.pixels), ids->rgb.pixels._length)  == -1) {
            d435_e_mem_detail d;
            snprintf(d.what, sizeof(d.what), "unable to allocate rgb frame memory");
            printf("d435: %s\n", d.what);
            return d435_e_mem(&d,self);
        }

        // Initialize sequence for ir frame
        ids->ir.height = streamIR->height();
        ids->ir.width = streamIR->width();
        ids->ir.bpp = 1;
        ids->ir.pixels._length = ids->ir.height*ids->ir.width*ids->ir.bpp;
        if (genom_sequence_reserve(&(ids->ir.pixels), ids->ir.pixels._length)  == -1) {
            d435_e_mem_detail d;
            snprintf(d.what, sizeof(d.what), "unable to allocate ir frame memory");
            printf("d435: %s\n", d.what);
            return d435_e_mem(&d,self);
        }

        // Initialize sequence for point cloud
        ids->pc.isRegistered = false;
        if (genom_sequence_reserve(&(ids->pc.points), 0)  == -1 ||
            genom_sequence_reserve(&(ids->pc.colors), 0)  == -1)
        {
            d435_e_mem_detail d;
            snprintf(d.what, sizeof(d.what), "unable to allocate point cloud memory");
            printf("d435: %s\n", d.what);
            return d435_e_mem(&d,self);
        }

        // Init boolean
        ids->pipe->init = true;

        ids->data = new d435_RSdata_s(ids->pipe->data);
    }
    else
    {
        ids->pipe->init = true;
        ids->started = true;
    }

    std::cout << "d435: initialization done" << std::endl;

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
d435_disconnect(d435_pipe_s **pipe, bool *started,
                const genom_context self)
{
    (*pipe)->init = false;
    *started = false;
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
d435_set_extrinsics(const sequence6_double *ext_values,
                    const d435_extrinsics *extrinsics,
                    const genom_context self)
{
    std::cout << "d435: new extrinsics calibration: ";
    d435_extrinsics_s ext;
    ext = {ext_values->_buffer[0],
           ext_values->_buffer[1],
           ext_values->_buffer[2],
           ext_values->_buffer[3],
           ext_values->_buffer[4],
           ext_values->_buffer[5]};

    *extrinsics->data(self) = ext;
    extrinsics->write(self);
    std::cout << extrinsics->data(self)->tx << " " <<
                 extrinsics->data(self)->ty << " " <<
                 extrinsics->data(self)->tz << " " <<
                 extrinsics->data(self)->roll << " " <<
                 extrinsics->data(self)->pitch << " " <<
                 extrinsics->data(self)->yaw << std::endl;

    return d435_ether;
}
