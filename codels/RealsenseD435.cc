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
 */

 #include "RealsenseD435.h"

 RealsenseD435::RealsenseD435(){

     // Start streaming
     auto prof = _pipe.start();

     // Set configuration as written in the .json calibration file
     rs2::device dev = prof.get_device();
     auto advanced = dev.as<rs400::advanced_mode>();
     std::ifstream t(calibfile);
     std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
     advanced.load_json(preset_json);

     // Setup for point cloud extraction
     // ptr_cloud cloud_raw (new pcl::PointCloud<pcl::PointXYZRGB> ());
     // ptr_cloud cloud_close(new pcl::PointCloud<pcl::PointXYZRGB> ());
     // ptr_cloud cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
     // int uu;
     // int vv;
     // rs2::pointcloud pc;
     // rs2::config cfg;
 }

 RealsenseD435::~RealsenseD435() {}
