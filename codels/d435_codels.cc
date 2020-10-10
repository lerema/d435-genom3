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

#include <err.h>
#include <cstdio>
#include <iostream>

/* --- Attribute set_format --------------------------------------------- */

/** Validation codel set_format of attribute set_format.
 *
 * Returns genom_ok.
 * Throws d435_e_io.
 */
genom_event
set_format(const char format[8], const genom_context self)
{
    if (!strcmp(format,"RGB8") ||
        !strcmp(format,"RGBA8") ||
        !strcmp(format,"Y16"))
        return genom_ok;
    else
    {
        d435_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "%s", "unallowed format");
        warnx("io error: %s", d.what);
        return d435_e_io(&d,self);
    }
}


/* --- Attribute set_fps ------------------------------------------------ */

/** Validation codel set_fps of attribute set_fps.
 *
 * Returns genom_ok.
 * Throws d435_e_io.
 */
genom_event
set_fps(uint16_t frequency, const genom_context self)
{
    if (frequency == 6 ||
        frequency == 15 ||
        frequency == 30 ||
        frequency == 60)
        return genom_ok;
    else
    {
        d435_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "%s", "unallowed frequency");
        warnx("io error: %s", d.what);
        return d435_e_io(&d,self);
    }
}


/* --- Attribute set_size ----------------------------------------------- */

/** Validation codel set_size of attribute set_size.
 *
 * Returns genom_ok.
 * Throws d435_e_io.
 */
genom_event
set_size(const or_camera_info_size_s *size, const genom_context self)
{
    if ((size->w == 1920 && size->h == 1080) ||
        (size->w == 1280 && size->h == 720) ||
        (size->w == 960 && size->h == 540) ||
        (size->w == 848 && size->h == 480) ||
        (size->w == 640 && size->h == 480) ||
        (size->w == 640 && size->h == 360) ||
        (size->w == 424 && size->h == 240) ||
        (size->w == 320 && size->h == 240))
        return genom_ok;
    else
    {
        d435_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "%s", "unallowed format");
        warnx("io error: %s", d.what);
        return d435_e_io(&d,self);
    }
}
