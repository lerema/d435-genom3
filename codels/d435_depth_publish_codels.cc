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
#include <sys/time.h>

/* --- Task depth_publish ----------------------------------------------- */


/** Codel d435_depth_start of task depth_publish.
 *
 * Triggered by d435_start.
 * Yields to d435_pause_start, d435_pub.
 */
genom_event
d435_depth_start(bool started, const genom_context self)
{
    // Loop in start until data is initialized, then yield to poll
    if (started)
        return d435_pub;
    else
        return d435_pause_start;
}


/** Codel d435_depth_pub of task depth_publish.
 *
 * Triggered by d435_pub.
 * Yields to d435_pause_start.
 */
genom_event
d435_depth_pub(const d435_RSdata_s *data, d435_pc_s *pc,
               const d435_pc_out *pc_out, const genom_context self)
{
    rs2::pointcloud rs_pc;
    rs2::points points = rs_pc.calculate(data->pc);
    // Tell pointcloud object to map to this color frame
    // rs_pc.map_to(data->rgb);

    if (points.size() == 0) {
        d435_e_rs_detail d;
        snprintf(d.what, sizeof(d.what), "empty point cloud");
        return d435_e_rs(&d,self);
    }

    // Create timestamp
    uint32_t ms = data->rgb.get_timestamp();
    uint32_t s = floor(ms/1000);
    uint64_t ns = (ms - s*1000) * 1e6;
    pc->ts.sec = s;
    pc->ts.nsec = ns;

    // Allocate memory for pc sequences
    if (genom_sequence_reserve(&(pc->points), points.size())  == -1) {
        d435_e_mem_detail d;
        snprintf(d.what, sizeof(d.what), "unable to allocate 3d point memory");
        return d435_e_mem(&d,self);
    }
    if (pc->isRegistered)
        if (genom_sequence_reserve(&(pc->colors), points.size())  == -1) {
            d435_e_mem_detail d;
            snprintf(d.what, sizeof(d.what), "unable to allocate points color memory");
            return d435_e_mem(&d,self);
        }

    // Get PC data
    const rs2::vertex *vertices = points.get_vertices();
    // const rs2::texture_coordinate *tex_coords = points.get_texture_coordinates();
    uint32_t valid_points = 0;
    uint8_t max_depth = 5;
    for (uint32_t i = 0; i < points.size(); i++)
    {
        if (vertices[i].z > 0.01 && std::abs(vertices[i].x) < max_depth && std::abs(vertices[i].y) < max_depth && vertices[i].z < max_depth)
        {
            pc->points._buffer[valid_points].x = vertices[i].x;
            pc->points._buffer[valid_points].y = vertices[i].y;
            pc->points._buffer[valid_points].z = vertices[i].z;
            // b[u,v] = pix[(u*w+v)*3] ; g[u,v] = pix[(u*w+v)*3+1] ; r[u,v] = pix[(u*w+v)*3+2]
            valid_points++;
        }
    }
    pc->length = valid_points;
    pc->points._length = valid_points;

    // Update port data
    *(pc_out->data(self)) = *pc;
    pc_out->write(self);

    return d435_pause_start;
}
