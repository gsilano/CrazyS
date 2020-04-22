/*
 * Copyright 2020 Ria Sonecha, Massachusetts Institute of Technology in Cambridge, MA, USA
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 #ifndef CRAZYFLIE_2_MELLINGER_CONTROLLER_H
 #define CRAZYFLIE_2_MELLINGER_CONTROLLER_H

 #include <boost/bind.hpp>
 #include <Eigen/Eigen>
 #include <stdio.h>

 #include <geometry_msgs/PoseStamped.h>
 #include <mav_msgs/Actuators.h>
 #include <mav_msgs/DroneState.h>
 #include <mav_msgs/AttitudeThrust.h>
 #include <mav_msgs/eigen_mav_msgs.h>
 #include <nav_msgs/Odometry.h>
 #include <ros/callback_queue.h>
 #include <ros/ros.h>
 #include <ros/time.h>

 #include "common.h"
 #include "controller_parameters_mellinger.h"
 #include "parameters.h"
 #include "stabilizer_types.h"

 namespace rotors_control {

     class MellingerController{
         public:
             MellingerController();
             ~MellingerController();
             void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

 	           void SetOdometryWithoutStateEstimator(const EigenOdometry& odometry);
             void SetTrajectoryPoint(const mav_msgs::EigenDroneState& command_trajectory);
             void SetControllerGains();
             void SetVehicleParameters();
             void SetSensorData(const sensorData_t& sensors);

             MellingerControllerParameters controller_parameters_mellinger_;
             VehicleParameters vehicle_parameters_;

         private:
             EIGEN_MAKE_ALIGNED_OPERATOR_NEW
             bool controller_active_;

             control_s control_t_;

             //Vehicle parameters
             double bf_, m_, g_;
             double l_, bm_;
             double Ix_, Iy_, Iz_;

             Eigen::Vector2f kpXYPositionController_, kdXYPositionController_, kiXYPositionController_;
             Eigen::Vector2f krXYPositionController_, kwXYPositionController_, ki_mXYPositionController_;
             Eigen::Vector2f iRangeMXY_, iRangeXY_, kdOmegaRP_;
             double kpZPositionController_, kdZPositionController_, kiZPositionController_;
             double krZPositionController_, kwZPositionController_, ki_mZPositionController_;
             double iRangeMZ_, iRangeZ_;
             double prev_setpoint_omega_pitch_, prev_setpoint_omega_roll_;
             double prev_omega_roll_, prev_omega_pitch_;
             double i_error_m_x_, i_error_m_y_, i_error_m_z_;
             double i_error_x_, i_error_y_, i_error_z_;

             mav_msgs::EigenDroneState command_trajectory_;
             EigenOdometry odometry_;
             state_t state_;
             sensorData_t sensors_;

             void GetMoments(double* u_2, double* u_3, double* u_4);
             void GetForce(double* u_1);
             void GetThrust(Eigen::Vector3f* target_thrust);
             void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;
             void Quaternion2EulerCommandTrajectory(double* roll, double* pitch, double* yaw) const;
             void Vnormalize(Eigen::Vector3f* vectorOutput, Eigen::Vector3f vectorInput);
             void Vcross(Eigen::Vector3f* vectorOutput, Eigen::Vector3f vectorA, Eigen::Vector3f vectorB);
             void Vsub(Eigen::Vector3f* vectorOutput, Eigen::Vector3f vectorA, point_t vectorB);
             void Vdot(double* output, Eigen::Vector3f vectorA, Eigen::Vector3f vectorB);
             void Clamp(double* value, double min, double max);
             void SetDroneState();
             void ControllerMellingerReset();


     };

 }
 #endif // CRAZYFLIE_2_MELLINGER_CONTROLLER_H
