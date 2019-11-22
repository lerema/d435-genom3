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
    rs2::pointcloud rs_pc;
    rs2::points points = rs_pc.calculate(data->depth);
    // Tell pointcloud object to map to this color frame
    // rs_pc.map_to(data->rgb);

    if (points.size() == 0) {
        d435_rs_error_detail d;
        snprintf(d.what, sizeof(d.what), "empty point cloud");
    }
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
        }
    }

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
