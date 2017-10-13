/*This file is part of dvrk-dynamics package.
 * Copyright (C) 2017, Giuseppe Andrea Fontanelli
 
 * Email id : giuseppeandrea.fontanelli@unina.it
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
* This code will subscriber integer values from demo_topic_publisher
*/

// Brief: da Vinci External force estimation

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>


#include <string.h>
#include "PSM_dynamics.h"

// set up joint state variables

std::vector<double> psm_joint_position;
std::vector<double> psm_joint_velocity;
std::vector<double> psm_joint_effort;

// psm joint feedback from RVIZ (joint_state_publisher)
void state_joint_current_cb(const sensor_msgs::JointState &msg)
{


        psm_joint_position = msg.position;  // outer_yaw_joint
        
        psm_joint_velocity = msg.velocity;  // outer_yaw_joint

        psm_joint_effort = msg.effort;  // outer_yaw_joint

    
}




int main(int argc, char** argv)
{

    std::cout << "Init" << std::endl;
    ros::init(argc, argv, "dvrk_psm_residual");

    ros::NodeHandle nh, nh_private("~");
    int freq = 200;
    ros::Rate rate(freq);  // 200 hz rate

    // subscriber
    std::cout << "Creo il subscriber" << std::endl;
    ros::Subscriber joint_state = nh.subscribe("/dvrk/PSM1/state_joint_current", 1, state_joint_current_cb);

    ros::Publisher external_forces_pub = nh.advertise<geometry_msgs::Wrench>("/dvrk/PSM1/external_wrench", 1);

    string LIB_D(LIB_DIRECTORY); 
    string param_D("/PSM_param.txt");

    PSM_dynamics psm_dyn("PSM1", LIB_D+param_D);
   

    Vector7d q = Vector7d::Zero();
    Vector7d dq = Vector7d::Zero();
    Vector6d tau = Vector6d::Zero();
    Vector6d qs = Vector6d::Zero();

    Vector6d In = Vector6d::Zero();
    Vector6d res = Vector6d::Zero();


    Vector6d dq_temp = Vector6d::Zero();
    Vector6d tau_temp = Vector6d::Zero();

    Matrix6d J_inv_T = Matrix6d::Zero();
    Matrix3d Jp_inv_T = Matrix3d::Zero();
    Matrix<double, 3,6> Jo_inv_T = Matrix<double, 3,6>::Zero();

    Matrix6d B = Matrix6d::Zero();
    Vector6d G = Vector6d::Zero();
    Vector7d F = Vector7d::Zero();
    Vector6d K = Vector6d::Zero();
    Matrix6d C = Matrix6d::Zero();
    Matrix6d J = Matrix6d::Zero();

    Matrix4d Te = Matrix4d::Zero();



    Vector3d external_forces = Vector3d::Zero();

    Vector3d external_torques = Vector3d::Zero();

    float Ki = 20; 


    float Tsam = 1/(float)freq;
    geometry_msgs::Wrench msg_wrench_ext;

    int count = 0;


    // ------------ run() --------------------------
    while (ros::ok()) {


        if (psm_joint_position.size() > 0){
            q  << psm_joint_position[0], psm_joint_position[1],psm_joint_position[2],psm_joint_position[3],psm_joint_position[4],psm_joint_position[5],psm_joint_position[6];
        }

        if (psm_joint_velocity.size() > 0){
        dq  << psm_joint_velocity[0],psm_joint_velocity[1],psm_joint_velocity[2],psm_joint_velocity[3],psm_joint_velocity[4],psm_joint_velocity[5],psm_joint_velocity[6];
        }

        if (psm_joint_effort.size() > 0){
        tau  << psm_joint_effort[0],psm_joint_effort[1],psm_joint_effort[2],psm_joint_effort[3],psm_joint_effort[4],psm_joint_effort[5],psm_joint_effort[6];
        }




        B = psm_dyn.PSM_B(q);
        G = psm_dyn.PSM_G(q,qs);

        K = psm_dyn.PSM_K(q);

        F = psm_dyn.PSM_F(dq);

        C = psm_dyn.PSM_C(q,dq);
        J = psm_dyn.PSM_J(q,qs);


        Te = psm_dyn.PSM_Te(q,qs);

        //LU<3,double> Core_Jo_T = J.slice<3,0,3,6>()*J.slice<3,0,3,6>().T();
		//LU<3,double> Core_Jp_T = J.slice<0,0,3,3>().T()*J.slice<0,0,3,3>();


        FullPivLU<Matrix3d> Core_Jo_T(J.block(3,0,3,6)*J.block(3,0,3,6).transpose());

        FullPivLU<Matrix3d> Core_Jp_T(J.block(0,0,3,3).transpose()*J.block(0,0,3,3));


        Jp_inv_T = J.block(0,0,3,3)*Core_Jp_T.inverse();
        Jo_inv_T = Core_Jo_T.inverse() * J.block(3,0,3,6);

   
        In = In + (tau + C.transpose()*dq.head(6) - F.head(6) - G - K + res)*Tsam;
        res = Ki*(B*dq.head(6) - In);
		
        external_forces = Jp_inv_T*res.head(3);
        external_torques = Jo_inv_T*res;
		
        msg_wrench_ext.force.x = external_forces[0];
        msg_wrench_ext.force.y = external_forces[1];
        msg_wrench_ext.force.z = external_forces[2];

        msg_wrench_ext.torque.x = external_torques[0];
        msg_wrench_ext.torque.y = external_torques[1];
        msg_wrench_ext.torque.z = external_torques[2];

        external_forces_pub.publish(msg_wrench_ext);

        count ++;
        ros::spinOnce();
        rate.sleep();
        
    }

    return 0;
}
