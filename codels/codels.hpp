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
#ifndef H_D435_CODELS
#define H_D435_CODELS

#include "acd435.h"

#include "d435_c_types.h"

#include <librealsense2/rs.hpp>
using namespace rs2;

#include <opencv2/opencv.hpp>
using namespace cv;


struct or_camera_pipe {
    pipeline       pipe;
    frameset       data;
};

// template <typename T>
// class custom_allocator {
// private:
//     T* mem;
//     size_t size;
// public:
//     typedef T     value_type;
//
//     custom_allocator(uint8_t* ptr, size_t n) : mem(ptr), size(n) {}
//     custom_allocator(const custom_allocator& other) throw() : mem(other.mem), size(other.size) {}
//
//     template <typename U>
//     custom_allocator &operator=(const custom_allocator<U> &other) { return *this; }
//     custom_allocator<T> &operator=(const custom_allocator &other) { return *this; }
//     ~custom_allocator() {}
//
//     T* allocate(size_t n, const void* hint = 0) { return mem; }
//     void deallocate(T* ptr, size_t n) {}
//     size_t max_size() const { return size; }
// };

#endif /* H_D435_CODELS */
