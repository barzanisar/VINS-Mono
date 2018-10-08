/*
 Copyright (c) 2015, Barza Nisar, Mina Kamel, ASL, ETH Zurich, Switzerland

 You can contact the author at <nisarb@student.ethz.ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <thread>
#include <chrono>
#include <Eigen/Core>
//#include <mav_msgs/conversions.h>
//#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <rosbag/bag.h>

#include <geometry_msgs/Vector3.h>
#include <mav_msgs/TorqueThrust.h>

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "torque_thrust_node");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher thrust_torque_pub =
      nh.advertise<mav_msgs::TorqueThrust>(
          "torque_thrust", 200);

  rosbag::Bag bag;
  bag.open("/home/barza/okvis/mav0/state_groundtruth_estimate0/test1.bag", rosbag::bagmode::Write);

  uint64_t imu_timestamp_ns;
  float Tx,Ty,Tz,Fz;

  ifstream infile("/home/barza/okvis/mav0/state_groundtruth_estimate0/t_imu_M_F_final.csv"); // for example
  
  string line = "";


  ROS_INFO("Start reference trajectory.");
  
  //uint64_t timeNanoSeconds = 1000;
  ros::Time timestamp;

  //mav_msgs::TorqueThrustPtr msg(new mav_msgs::TorqueThrust);
  mav_msgs::TorqueThrust msg;

  while (getline(infile, line)) {
    stringstream strstr(line);
    string word = "";
    
    getline(strstr,word, ',');
    stringstream(word) >> imu_timestamp_ns;


    getline(strstr,word, ',');
    stringstream(word) >> Tx;

    getline(strstr,word, ',');
    stringstream(word) >> Ty;

    getline(strstr,word, ',');
    stringstream(word) >> Tz;

    getline(strstr,word);
    stringstream(word) >> Fz;


      msg.header.stamp = timestamp.fromNSec(imu_timestamp_ns);
      msg.thrust.x=0;
      msg.thrust.y=0;
      msg.thrust.z=Fz;
      msg.torque.x=Tx;
      msg.torque.y=Ty;
      msg.torque.z=Tz;

      ros::Duration(0.0048).sleep(); 

      thrust_torque_pub.publish(msg);

      bag.write("torque_thrust", ros::Time::now(), msg);

  }

  ROS_INFO("End reference trajectory.");

  //ros::spinOnce();
  
  bag.close();	
  ros::spin();
  //ros::shutdown();

  return 0;
}
