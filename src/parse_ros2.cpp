/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef OV_AUTO_PARSE_ROSHANDLER_H
#define OV_AUTO_PARSE_ROSHANDLER_H


#include <ros/ros.h>

#include "option/SystemOptions.h"
#include "ov_core/types/LandmarkRepresentation.h"
#include "utils/nh_wrapper.h"


namespace ov_auto {


    /**
     * @brief This function will load paramters from the ros node handler / paramter server
     * This is the recommended way of loading parameters as compared to the command line version.
     * @param nh ROS node handler
     * @return A fully loaded SystemOptions object
     */
    SystemOptions parse_ros_nodehandler(ros::NodeHandle &nh) {

        // Our vio manager options with defaults
        SystemOptions op;
        nh_wrapper cnh;


        // Success, lets returned the parsed options
        return op;

    }


}


#endif //OV_AUTO_PARSE_ROSHANDLER_H