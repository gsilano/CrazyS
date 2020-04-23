/*
 * Copyright 2020 Ria Sonecha, MIT, USA
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 #ifndef HOVERING_EXAMPLE_SPLINE_H
 #define HOVERING_EXAMPLE_SPLINE_H

 #include <boost/bind.hpp>
 #include <Eigen/Eigen>
 #include <Eigen/Core>
 #include <math.h>
 #include <stdio.h>

 // package libraries
 #include "rotors_gazebo/Matrix3x3.h"
 #include "rotors_gazebo/Quaternion.h"
 #include "rotors_gazebo/transform_datatypes.h"

 #include <nav_msgs/Odometry.h>
 #include <mav_msgs/conversions.h>
 #include <mav_msgs/default_topics.h>
 #include <sensor_msgs/Imu.h>
 #include <ros/callback_queue.h>
 #include <ros/ros.h>
 #include <ros/time.h>
 #include <mav_msgs/DroneState.h>
 #include <mav_msgs/eigen_mav_msgs.h>


 namespace rotors_gazebo {

     class SplineTrajectoryGenerator{
         public:

           SplineTrajectoryGenerator();
           ~SplineTrajectoryGenerator();

           void TrajectoryCallback(mav_msgs::EigenDroneState* odometry, double* time_final, double* time_init);
           void InitializeParams();
           template<typename T> inline void GetRosParameterHovering(const ros::NodeHandle& nh,
                                                            const std::string& key,
                                                            const T& default_value,
                                                            T* value);

         private:
             bool enable_parameter_computation_ = true;
             //polynomial coefficients
             Eigen::Vector3f a0_, a1_, a2_, a3_, a4_, a5_;
             Eigen::Vector3f b0_, b1_, b2_, b3_, b4_;
             Eigen::Vector3f c0_, c1_, c2_, c3_;

             Eigen::Vector3f g0_, g1_, g2_, g3_, g4_, g5_;
             Eigen::Vector3f h0_, h1_, h2_, h3_, h4_;
             Eigen::Vector3f i0_, i1_, i2_, i3_;

             // Desidred drone orientation expressed in radians
             double rollDesRad_, pitchDesRad_, yawDesRad_;

             // Parameters used for the trajectory generation
             double time_final_;
             Eigen::Vector3f position_initial_, position_final_, velocity_initial_, velocity_final_, acceleration_initial_, acceleration_final_;
             Eigen::Vector3f orientation_initial_, orientation_final_, angular_velocity_initial_, angular_velocity_final_, angular_acceleration_initial_, angular_acceleration_final_;

             void ComputeSplineParameters(double* time_spline);

             void Euler2QuaternionCommandTrajectory(double* x, double* y, double* z, double* w) const;

     };

 }
 #endif // HOVERING_EXAMPLE_SPLINE_H
