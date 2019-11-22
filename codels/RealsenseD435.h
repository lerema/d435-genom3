/*
 * Copyright (c) 2019-2020 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 *                                      Antonio Enrique Jimenez Cano on Mon Oct 16 2019
 */
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <math.h>

#include "Eigen/Core"
#include "Eigen/Dense"

using namespace Eigen;

class RealsenseD435
{
private:

    rs2::pipeline _pipe;
    rs2::config _cfg;

public:

    RealsenseD435();
    ~RealsenseD435();

};
