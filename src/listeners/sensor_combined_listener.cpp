/****************************************************************************
 *
 * Copyright 2018 PX4 Pro Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Sensor Combined uORB topic listener example
 * @file sensor_combined_listener.cpp
 * @addtogroup ros1_examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include "ros/ros.h"

#include <boost/thread.hpp>
#include <boost/make_unique.hpp>

#include "px4_msgs/SensorCombined.h"

static constexpr auto kDefaultSensorCombinedPubTopic = "SensorCombined_topic_feedback";
static constexpr auto kDefaultSensorCombinedSubTopic = "SensorCombined_topic";

//! sensor_combined publisher (to ROS) naming
std::string _sensor_combined_pub_topic;
//! sensor_combined subscriber (from ROS2) naming
std::string _sensor_combined_sub_topic;

//! sensor_combined publisher (to ROS)
ros::Publisher _sensor_combined_pub;
//! sensor_combined subscriber (from ROS2)
ros::Subscriber _sensor_combined_sub;

/**
 * @brief Sensor Combined uORB topic data callback
 */
void sensor_combined_callback(const px4_msgs::SensorCombined::ConstPtr& msg)
{
	ROS_INFO("Receiving sensor_combined_topic");
	std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
	std::cout << "RECEIVED DATA FROM SENSOR COMBINED" << std::endl;
	std::cout << "================================" << std::endl;
	std::cout << "gyro_rad[0]: " << msg->gyro_rad[0] << std::endl;
	std::cout << "gyro_rad[1]: " << msg->gyro_rad[1] << std::endl;
	std::cout << "gyro_rad[2]: " << msg->gyro_rad[2] << std::endl;
	std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
	std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
	std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
	std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
	std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
	std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;
	std::cout << std::endl << "Publishing back..." << std::endl;
	fflush(stdout);

        // republish data in another topic. Can be converted to a std ROS msg instead
	_sensor_combined_pub.publish(msg);
}

int main(int argc, char **argv)
{
        // init the node
	ros::init(argc, argv, "sensor_combined_listener_node");

        // create the node handler
        ros::NodeHandle nh;

        // set the publisher
        _sensor_combined_pub = nh.advertise<px4_msgs::SensorCombined>(kDefaultSensorCombinedPubTopic, 100);

        // set the subscriber
        _sensor_combined_sub = nh.subscribe(kDefaultSensorCombinedSubTopic, 100, sensor_combined_callback);

        // sping the node
        ros::spin();

	return 0;
}
