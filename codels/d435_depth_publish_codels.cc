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
#include "t3d/t3d.h"

/* --- Task depth_publish ----------------------------------------------- */


/** Codel d435_depth_start of task depth_publish.
 *
 * Triggered by d435_start.
 * Yields to d435_pub.
 */
genom_event
d435_depth_start(const d435_RSdata_s *data, const genom_context self)
{
    // Loop in start until data is initialized, then yield to poll
    if (data->init)
        return d435_pub;
    else
        return d435_start;
}


/** Codel d435_depth_pub of task depth_publish.
 *
 * Triggered by d435_pub.
 * Yields to d435_pause_pub.
 */
genom_event
d435_depth_pub(const d435_RSdata_s *data, d435_pc_s *pc,
               const d435_curr_pc *curr_pc, const genom_context self)
{
    // Read time from data struct
    const unsigned long ms = data->depth.get_timestamp();
    const uint8_t *rgb_data = data->rgb.get_data();
    const uint w = data->rgb.get_width();

    rs2::pointcloud rs_pc;
    rs2::points points = rs_pc.calculate(data->depth);
    // Tell pointcloud object to map to this color frame
    // rs_pc.map_to(data->rgb);

    if (points.size() == 0) {
        d435_rs_error_detail d;
        snprintf(d.what, sizeof(d.what), "empty point cloud");
    }

    // Create timestamp
    // If using time of publishing
    // struct timeval tv;
    // gettimeofday(&tv, NULL);
    // frame->timeStamp.sec = tv.tv_sec;
    // frame->timeStamp.nsec = tv.tv_usec * 1000;

    // Else if using time of capture
    unsigned long s = floor(ms/1000);
    unsigned long long ns = (ms - s*1000) * 1e6;
    pc->timeStamp.sec = s;
    pc->timeStamp.nsec = ns;

    if (genom_sequence_reserve(&(pc->points), points.size())  == -1) {
        d435_mem_error_detail d;
        snprintf(d.what, sizeof(d.what), "unable to allocate 3d point memory");
    }
    if (genom_sequence_reserve(&(pc->colors), points.size())  == -1) {
        d435_mem_error_detail d;
        snprintf(d.what, sizeof(d.what), "unable to allocate points color memory");
    }
    // Get PC data
    const rs2::vertex *vertices = points.get_vertices();
    // const rs2::texture_coordinate *tex_coords = points.get_texture_coordinates();

    // Iterate over PC
    for (uint i = 0; i < points.size(); i++)
    {
        if (vertices[i].z)
        {
            pc->points._buffer[i].x = vertices[i].x;
            pc->points._buffer[i].y = vertices[i].y;
            pc->points._buffer[i].z = vertices[i].z;
            // b[u,v] = pix[(u*w+v)*3] ; g[u,v] = pix[(u*w+v)*3+1] ; r[u,v] = pix[(u*w+v)*3+2]
        }
    }

    // Update port data
    *(curr_pc->data(self)) = *pc;
    curr_pc->write(self);

    return d435_pause_pub;
}


/** Codel d435_depth_stop of task depth_publish.
 *
 * Triggered by d435_stop.
 * Yields to d435_ether.
 */
genom_event
d435_depth_stop(const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return d435_ether;
}
