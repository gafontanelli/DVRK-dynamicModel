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

#include <TooN/TooN.h>
#include <TooN/LU.h>

#include <string.h>


#include "dvrk_dynamics/MTM_dynamics.h"

#define mtmName MTML

using namespace TooN;

// set up joint state variables

std::vector<double> mtm_joint_position;
std::vector<double> mtm_joint_velocity;
std::vector<double> mtm_joint_effort;

Vector<3> xd = Zeros;

// psm joint feedback (joint_state_publisher)
void state_joint_current_cb(const sensor_msgs::JointState &msg)
{
        mtm_joint_position = msg.position;  // outer_yaw_joint
        
        mtm_joint_velocity = msg.velocity;  // outer_yaw_joint

        mtm_joint_effort = msg.effort;  // outer_yaw_joint
}



int main(int argc, char** argv)
{
    std::cout << "dvrk_mtm_residual" << std::endl;
    ros::init(argc, argv, "dvrk_MTM_residual");
    ros::NodeHandle nh, nh_private("~");
    int freq = 200;
    ros::Rate rate(freq);  // 200 hz rate

    ros::Subscriber joint_state = nh.subscribe("/dvrk/MTML/state_joint_current", 1, state_joint_current_cb);
    ros::Publisher external_forces_pub = nh.advertise<geometry_msgs::Wrench>("/dvrk/MTML/external_wrench", 1);

    MTM_dynamics mtm_dyn(3);

    Vector<7> q = Zeros;
    Vector<7> dq = Zeros;
    Vector<7> tau = Zeros;

    Vector<7> In = Zeros;
    Vector<7> res = Zeros;

    Vector<6> dq_temp = Zeros;
    Vector<6> tau_temp = Zeros;
	
	Matrix<3,3> Jp_inv_T = Zeros;
    Matrix<3,7> Jo_inv_T = Zeros;
    Matrix<4> Te_inv = Zeros;

    Matrix <7,7> B = Zeros;
    Vector <7> G = Zeros;
    Vector <7> F = Zeros;
    Vector <7> K = Zeros;
    Matrix <7,7> C = Zeros;
    Matrix <6,7> J = Zeros;

    Matrix<3,3> dJ3 = Zeros;
    Matrix<4> Te = Zeros;

    Vector<3> external_forces = Zeros;
	Vector<3> external_torques = Zeros;


    float Ki = 5; 

    float Tsam = 1/(float)freq;

    geometry_msgs::Wrench msg_wrench_ext;


    int count = 0;

    // ------------ run() --------------------------
    while (ros::ok()) {


        if (mtm_joint_position.size() > 0){
        q = makeVector(mtm_joint_position[0],mtm_joint_position[1],mtm_joint_position[2],mtm_joint_position[3],mtm_joint_position[4],mtm_joint_position[5],mtm_joint_position[6]);
        }

        if (mtm_joint_velocity.size() > 0){
        dq = makeVector(mtm_joint_velocity[0],mtm_joint_velocity[1],mtm_joint_velocity[2],mtm_joint_velocity[3],mtm_joint_velocity[4],mtm_joint_velocity[5],mtm_joint_velocity[6]);
        }

        if (mtm_joint_effort.size() > 0){
        tau = makeVector(mtm_joint_effort[0],mtm_joint_effort[1],mtm_joint_effort[2],mtm_joint_effort[3],mtm_joint_effort[4],mtm_joint_effort[5],mtm_joint_effort[6]);
        }

        B = mtm_dyn.MTM_B(q);
        G = mtm_dyn.MTM_G(q);
        K = mtm_dyn.MTM_K(q);
        F = mtm_dyn.MTM_F(dq);
        C = mtm_dyn.MTM_C(q,dq);

        J = mtm_dyn.MTM_J(q);
        Te = mtm_dyn.MTM_Te(q);
        dJ3 = mtm_dyn.MTM_dJ3(q,dq);


        LU<3,double> Core_Jo_T = J.slice<3,0,3,7>()*J.slice<3,0,3,7>().T();
		LU<3,double> Core_Jp_T = J.slice<0,0,3,3>().T()*J.slice<0,0,3,3>();

        Jp_inv_T = J.slice<0,0,3,3>()*Core_Jp_T.get_inverse();
        Jo_inv_T = Core_Jo_T.get_inverse() * J.slice<3,0,3,7>();

  
        In = In + (tau + C.T()*dq - F - G - K + res)*Tsam; 
        res = Ki*(B*dq - In);

        external_forces = Jp_inv_T*makeVector(res[0],res[1],res[2]);
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
