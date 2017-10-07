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

//RodyMan_kinematics.cpp
#include <stdio.h>
#include <math.h>
#include "PSM_dynamics.h"


#ifndef M_PI
#define M_PI 3.14159265358979
#endif


//******************************************************************************
PSM_dynamics::PSM_dynamics(int n)
{

}

// Inertia matrix B
Matrix6d PSM_dynamics::PSM_B(Vector7d q, Vector6d qs){

Matrix6d A0 = Matrix6d::Zero();
double q1 = q[0];
double q2 = q[1];
double q3 = q[2];
double q4 = q[3];
double q5 = q[4];
double q6 = q[5];
double qs5 = qs[4];
double qs6 = qs[5];

double t2 = cos(q2);
double t3 = t2*t2;
double t4 = t3-1.0;
double t5 = q2*2.0;
double t6 = sin(t5);
double t7 = sin(q2);
double t8 = Ixy4*t2;
double t9 = Ixy5*t2;
double t10 = mpz3*t7*(1.0/5.0);
double t11 = q3*2.5E3;
double t12 = t11-3.9E1;
double t13 = q3*5.0;
double t14 = t13+1.0;
A0(0,0) = Ixx3+Iyy1-Ixx2*t4+Ixx4*t3+Ixx5*t3+Ixx9*t3-Ixz4*t6-Ixz5*t6+Iyy2*t3-Izz4*t4-Izz5*t4-Izz9*t4+mass3*t3*(1.0/2.5E1)+mass4*t3*(1.0/2.5E1)-mpy3*t2*(2.0/5.0)-mpz4*t3*(2.0/5.0)-mpz5*(t3*3.12E-2-q3*t3*2.0)-mpx5*(t6*1.56E-2-q3*t6)-mpx4*t2*t7*(2.0/5.0)+mass9*t14*(t3*2.0E1+q3*t3*1.0E2)*(1.0/5.0E2)-mass5*t12*(t3*3.9E1-q3*t3*2.5E3)*1.6E-7;
A0(0,1) = t8+t9+t10-Iyz4*t7-Iyz5*t7-mpy4*t7*(1.0/5.0)-mpy5*(t7*1.56E-2-q3*t7);
A0(0,2) = -mpy5*t2;
A0(1,0) = t8+t9+t10-Iyz4*t7-Iyz5*t7-mpy4*t7*(1.0/5.0)+mpy5*t7*t12*4.0E-4;
A0(1,1) = Iyy4+Iyy5+Iyy9+Izz2+mass3*(1.0/2.5E1)+mass4*(1.0/2.5E1)-mpz4*(2.0/5.0)+mpz5*(q3*2.0-3.12E-2)+mass9*t14*(q3*1.0E2+2.0E1)*(1.0/5.0E2)+mass5*t12*(q3*5.0E3-7.8E1)*8.0E-8;
A0(1,2) = mpx5;
A0(2,0) = -mpy5*t2;
A0(2,1) = mpx5;
A0(2,2) = mass5+mass9;
return A0;
}

//Coriolis and centrifugal matrix C
Matrix6d PSM_dynamics::PSM_C(Vector7d q, Vector7d dq, Vector6d qs){

Matrix6d A0 = Matrix6d::Zero();

double q1 = q[0];
double q2 = q[1];
double q3 = q[2];
double q4 = q[3];
double q5 = q[4];
double q6 = q[5];

double dq1 = dq[0];
double dq2 = dq[1];
double dq3 = dq[2];
double dq4 = dq[3];
double dq5 = dq[4];
double dq6 = dq[5];

double qs5 = qs[4];
double qs6 = qs[5];

double t2 = q2*2.0;
double t3 = cos(t2);
double t4 = sin(t2);
double t5 = q3*q3;
double t6 = sin(q2);
double t7 = cos(q2);
double t8 = t7*t7;
double t9 = mass9*5.0E2;
double t10 = mpz5*2.5E3;
double t11 = mass5*q3*2.5E3;
double t12 = mass9*q3*2.5E3;
double t13 = mass5*-3.9E1+t9+t10+t11+t12;
A0(0,0) = dq3*mass5*(-1.56E-2)+dq3*mass9*(1.0/5.0)+dq3*mpz5+Ixx2*dq2*t4-Ixx4*dq2*t4-Ixx5*dq2*t4-Ixx9*dq2*t4-Ixz4*dq2*t3*2.0-Ixz5*dq2*t3*2.0-Iyy2*dq2*t4+Izz4*dq2*t4+Izz5*dq2*t4+Izz9*dq2*t4+dq3*mass5*q3+dq3*mass9*q3-dq2*mass3*t4*(1.0/2.5E1)-dq2*mass4*t4*(1.0/2.5E1)-dq2*mass5*t4*2.4336E-4-dq3*mass5*t3*1.56E-2-dq2*mass9*t4*(1.0/2.5E1)+dq3*mass9*t3*(1.0/5.0)-dq2*mpx4*t3*(2.0/5.0)-dq2*mpx5*t3*3.12E-2+dq3*mpx5*t4+dq2*mpy3*t6*(2.0/5.0)+dq2*mpz4*t4*(2.0/5.0)+dq2*mpz5*t4*3.12E-2+dq3*mpz5*t3+dq2*mass5*q3*t4*3.12E-2+dq3*mass5*q3*t3-dq2*mass9*q3*t4*(2.0/5.0)+dq3*mass9*q3*t3+dq2*mpx5*q3*t3*2.0-dq2*mpz5*q3*t4*2.0-dq2*mass5*t4*t5-dq2*mass9*t4*t5;
A0(0,1) = Ixx2*dq1*t4-Ixx4*dq1*t4-Ixx5*dq1*t4-Ixx9*dq1*t4-Ixz4*dq1*t3*2.0-Ixz5*dq1*t3*2.0-Iyy2*dq1*t4+Izz4*dq1*t4+Izz5*dq1*t4+Izz9*dq1*t4-dq1*mass3*t4*(1.0/2.5E1)-dq1*mass4*t4*(1.0/2.5E1)-dq1*mass5*t4*2.4336E-4-dq1*mass9*t4*(1.0/2.5E1)-dq1*mpx4*t3*(2.0/5.0)-dq1*mpx5*t3*3.12E-2+dq1*mpy3*t6*(2.0/5.0)+dq3*mpy5*t6*2.0+dq1*mpz4*t4*(2.0/5.0)+dq1*mpz5*t4*3.12E-2+dq1*mass5*q3*t4*3.12E-2-dq1*mass9*q3*t4*(2.0/5.0)+dq1*mpx5*q3*t3*2.0-dq1*mpz5*q3*t4*2.0-dq1*mass5*t4*t5-dq1*mass9*t4*t5;
A0(0,2) = dq1*mass5*t8*(-3.12E-2)+dq1*mass9*t8*(2.0/5.0)+dq2*mpy5*t6*2.0+dq1*mpz5*t8*2.0+dq1*mass5*q3*t8*2.0+dq1*mass9*q3*t8*2.0+dq1*mpx5*t6*t7*2.0;
A0(1,0) = Ixx2*t4*(-1.0/2.0)+Ixx4*t4*(1.0/2.0)+Ixx5*t4*(1.0/2.0)+Ixx9*t4*(1.0/2.0)+Ixz4*t3+Ixz5*t3+Iyy2*t4*(1.0/2.0)-Izz4*t4*(1.0/2.0)-Izz5*t4*(1.0/2.0)-Izz9*t4*(1.0/2.0)+mass3*t4*(1.0/5.0E1)+mass4*t4*(1.0/5.0E1)+mass5*t4*1.2168E-4+mass9*t4*(1.0/5.0E1)+mpx4*t3*(1.0/5.0)+mpx5*t3*1.56E-2-mpy3*t6*(1.0/5.0)-mpz4*t4*(1.0/5.0)-mpz5*t4*1.56E-2-mass5*q3*t4*1.56E-2+mass9*q3*t4*(1.0/5.0)-mpx5*q3*t3+mpz5*q3*t4+mass5*t4*t5*(1.0/2.0)+mass9*t4*t5*(1.0/2.0);
A0(1,1) = dq3*t13*8.0E-4;
A0(1,2) = dq2*t13*8.0E-4;
A0(2,0) = mass5*t8*1.56E-2-mass9*t8*(1.0/5.0)-mpx5*t4*(1.0/2.0)-mpz5*t8-mass5*q3*t8-mass9*q3*t8;
A0(2,1) = mass5*1.56E-2-mass9*(1.0/5.0)-mpz5-mass5*q3-mass9*q3;


return A0;
}

//Friction Vector F
Vector7d PSM_dynamics::PSM_F(Vector7d dq){

Vector7d A0 = Vector7d::Zero();

float dq1 = dq[0];
float dq2 = dq[1];
float dq3 = dq[2];
float dq4 = dq[3];
float dq5 = dq[4];
float dq6 = dq[5];
float dq7 = dq[6];
/*if ( dq1> 0.001 ){ t1 = 1; }else if( dq1<-0.001){ t1 = -1; }else{ t1 = 0; }
if ( dq2> 0.001 ){ t2 = 1; }else if( dq2<-0.001){ t2 = -1; }else{ t2 = 0; }
if ( dq3> 0.001 ){ t3 = 1; }else if( dq3<-0.001){ t3 = -1; }else{ t3 = 0; }
if ( dq4> 0.001 ){ t4 = 1; }else if( dq4<-0.001){ t4 = -1; }else{ t4 = 0; }
if ( dq5> 0.001 ){ t5 = 1; }else if( dq5<-0.001){ t5 = -1; }else{ t5 = 0; }
if ( dq6> 0.001 ){ t6 = 1; }else if( dq6<-0.001){ t6 = -1; }else{ t6 = 0; }
// t1 =0;
// t2 =0;
// t3 =0;
// t4 =0;
// t5 =0;
// t6 =0;

A0[0] = Fv11*dq1+Fs11*(t1);
A0[1] = Fv22*dq2+Fs22*(t2);
A0[2] = Fv33*dq3+Fs33*(t3);
A0[3] = Fv44*dq4+Fs44*(t4);
A0[4] = Fv55*dq5+Fv56*dq6+Fs55*(t5);
A0[5] = Fv65*dq5+Fv66*dq6+Fs66*(t6);//*/


float friction_slope = 50;


float t2 = dq5*6.696E-1;
float t3 = dq6*8.212E-1;
float t4 = dq7*4.106E-1;
float t5 = t2+t3+t4;
float t6 = tanh(friction_slope*t5);//(t5/fabs(t5));
float t7 = t2+t3-t4;
float t8 = tanh(friction_slope*t7);//(t7/fabs(t7));
float t9 = dq7*(-4.106E-1)+t2+t3;
float t10 = tanh(friction_slope*t9);



A0[0] = Fv1*dq1+Fs1*(tanh(friction_slope*dq1));
A0[1] = Fv2*dq2+Fs2*(tanh(friction_slope*dq2));
A0[2] = Fv3*dq3+Fs3*(tanh(friction_slope*dq3));
A0[3] = Fv4*dq4*4.0921609E-1+Fs4*(tanh(friction_slope*dq4))*6.397E-1;
A0[4] = Fv5*dq5*9.6373489E-1+Fv6*dq5*4.4836416E-1+Fv6*dq6*5.4987552E-1+Fv7*dq5*4.4836416E-1-Fv6*dq7*2.7493776E-1+Fv7*dq6*5.4987552E-1+Fv7*dq7*2.7493776E-1+Fs7*t6*6.696E-1+Fs6*(t10)*6.696E-1+Fs5*(tanh(friction_slope*dq5))*9.817E-1;
A0[5] = Fv6*dq5*5.4987552E-1+Fv6*dq6*6.7436944E-1+Fv7*dq5*5.4987552E-1-Fv6*dq7*3.3718472E-1+Fv7*dq6*6.7436944E-1+Fv7*dq7*3.3718472E-1+Fs7*t6*8.212E-1+Fs6*t8*8.212E-1;
A0[6] = Fv6*dq5*(-2.7493776E-1)-Fv6*dq6*3.3718472E-1+Fv7*dq5*2.7493776E-1+Fv6*dq7*1.6859236E-1+Fv7*dq6*3.3718472E-1+Fv7*dq7*1.6859236E-1+Fs7*t6*4.106E-1-Fs6*t8*4.106E-1;

return A0;
}

//Gravity Vector G
Vector6d PSM_dynamics::PSM_G(Vector7d q, Vector6d qs){

Vector6d A0 = Vector6d::Zero();

double q1 = q[0];
double q2 = q[1];
double q3 = q[2];
double q4 = q[3];
double q5 = q[4];
double q6 = q[5];

double qs5 = qs[4];
double qs6 = qs[5];

double t2 = cos(qs5);
double t3 = sin(q1);
double t4 = cos(q1);
double t5 = cos(qs6);
double t6 = sin(qs5);
double t7 = t2*t4*(9.81E2/1.0E2);
double t8 = t3*t5*t6*(9.81E2/1.0E2);
double t9 = t7+t8;
double t10 = t2*t3;
double t13 = t4*t5*t6;
double t11 = t10-t13;
double t12 = cos(q2);
double t14 = sin(q2);
double t15 = sin(qs6);
double t16 = t6*t14*t15*(9.81E2/1.0E2);
double t17 = t2*t4*t12*(9.81E2/1.0E2);
double t18 = t3*t5*t6*t12*(9.81E2/1.0E2);
double t19 = t16+t17+t18;
double t20 = t2*t4*t14*(9.81E2/5.0E2);
double t21 = t3*t5*t6*t14*(9.81E2/5.0E2);
double t22 = t20+t21-t6*t12*t15*(9.81E2/5.0E2);
double t23 = t2*t4*t14*(9.81E2/1.0E2);
double t24 = t3*t5*t6*t14*(9.81E2/1.0E2);
double t26 = t6*t12*t15*(9.81E2/1.0E2);
double t25 = t23+t24-t26;
double t27 = q3*5.0;
double t28 = t27+1.0;
double t29 = q3*2.5E3;
double t30 = t29-3.9E1;
A0[0] = mpy3*(t2*t3*(9.81E2/1.0E2)-t4*t5*t6*(9.81E2/1.0E2))-mpx1*t11*(9.81E2/1.0E2)+mpy4*t9+mpy5*t9-mpz1*t9-mpz2*t9-mpz3*t9-mass3*t11*t12*(9.81E2/5.0E2)-mass4*t11*t12*(9.81E2/5.0E2)-mpx2*t11*t12*(9.81E2/1.0E2)+mpx4*t11*t14*(9.81E2/1.0E2)+mpx5*t11*t14*(9.81E2/1.0E2)+mpy2*t11*t14*(9.81E2/1.0E2)+mpz4*t11*t12*(9.81E2/1.0E2)+mpz5*t11*t12*(9.81E2/1.0E2)+mass5*t11*t12*t30*3.924E-3-mass9*t11*t12*t28*(9.81E2/5.0E2);
A0[1] = -mass3*t22-mass4*t22-mpx4*t19-mpx5*t19-mpx2*t25-mpy2*t19+mpz4*t25+mpz5*t25-mass9*t28*(t2*t4*t14*9.81E2-t6*t12*t15*9.81E2+t3*t5*t6*t14*9.81E2)*(1.0/5.0E2)+mass5*t30*(t2*t4*t14*4.905E4-t6*t12*t15*4.905E4+t3*t5*t6*t14*4.905E4)*8.0E-8;
A0[2] = (mass5-mass9)*(t2*t4*t12+t6*t14*t15+t3*t5*t6*t12)*(-9.81E2/1.0E2);
return A0;
}

//Elasticity Vector K
Vector6d PSM_dynamics::PSM_K(Vector7d q){


double q1 = q[0];
double q2 = q[1];
double q3 = q[2];
double q4 = q[3];
double q5 = q[4];
double q6 = q[5];

Vector6d A0 = Vector6d::Zero();
A0[0] = O1+K1*q1;
A0[1] = O2+K2*q2;
A0[3] = O4+K4*q4;

return A0;
}


//Dyrect Kinematics Matrix Te
Matrix4d PSM_dynamics::PSM_Te(Vector7d q, Vector6d qs){

Matrix4d A0 = Matrix4d::Zero();
double q1 = q[0];
double q2 = q[1];
double q3 = q[2];
double q4 = q[3];
double q5 = q[4];
double q6 = q[5];

double qs5 = qs[4];
double qs6 = qs[5];

double t2 = cos(qs5);
double t3 = sin(q2);
double t4 = cos(q1);
double t5 = sin(qs5);
double t6 = t4*t5;
double t7 = cos(qs6);
double t8 = sin(q1);
double t13 = t2*t7*t8;
double t9 = t6-t13;
double t10 = cos(q2);
double t11 = sin(qs6);
double t12 = sin(q4);
double t14 = t3*t9;
double t15 = t2*t10*t11;
double t16 = t14+t15;
double t17 = cos(q4);
double t18 = t5*t8;
double t19 = t2*t4*t7;
double t20 = t18+t19;
double t21 = sin(q5);
double t22 = t9*t10;
double t27 = t2*t3*t11;
double t23 = t22-t27;
double t24 = cos(q5);
double t25 = t12*t20;
double t26 = cos(q6);
double t28 = t23*t24;
double t35 = t16*t17;
double t36 = t25-t35;
double t29 = t21*t36;
double t30 = t28+t29;
double t31 = sin(q6);
double t32 = t12*t16;
double t33 = t17*t20;
double t34 = t32+t33;
double t37 = t7*t10;
double t38 = t3*t8*t11;
double t39 = t37+t38;
double t40 = t17*t39;
double t41 = t4*t11*t12;
double t42 = t40+t41;
double t43 = t3*t7;
double t48 = t8*t10*t11;
double t44 = t43-t48;
double t45 = t12*t39;
double t46 = t45-t4*t11*t17;
double t47 = t21*t42;
double t49 = t24*t44;
double t50 = t47+t49;
double t51 = q3-1.56E-2;
double t52 = t2*t4;
double t53 = t5*t7*t8;
double t54 = t52+t53;
double t55 = t3*t54;
double t62 = t5*t10*t11;
double t56 = t55-t62;
double t57 = t2*t8;
double t63 = t4*t5*t7;
double t58 = t57-t63;
double t59 = t10*t54;
double t60 = t3*t5*t11;
double t61 = t59+t60;
double t64 = t12*t58;
double t65 = t24*t61;
double t71 = t17*t56;
double t72 = t64-t71;
double t66 = t21*t72;
double t67 = t65+t66;
double t68 = t12*t56;
double t69 = t17*t58;
double t70 = t68+t69;
A0(0,0) = -t26*t34-t30*t31;
A0(0,1) = -t21*t23+t24*(t25-t16*t17);
A0(0,2) = -t26*t30+t31*t34;
A0(0,3) = t23*t24*(-9.1E-3)-t21*t36*9.1E-3-t23*t51;
A0(1,0) = t26*t46-t31*t50;
A0(1,1) = -t21*t44+t24*t42;
A0(1,2) = -t26*t50-t31*t46;
A0(1,3) = t21*t42*(-9.1E-3)-t24*t44*9.1E-3-t44*t51;
A0(2,0) = -t26*t70-t31*t67;
A0(2,1) = -t21*t61+t24*(t64-t17*t56);
A0(2,2) = -t26*t67+t31*t70;
A0(2,3) = t24*t61*(-9.1E-3)-t21*t72*9.1E-3-t51*t61;
A0(3,3) = 1.0;
return A0;
}


//Jacobian matrix J
Matrix6d PSM_dynamics::PSM_J(Vector7d q, Vector6d qs){

Matrix6d A0 = Matrix6d::Zero();
double q1 = q[0];
double q2 = q[1];
double q3 = q[2];
double q4 = q[3];
double q5 = q[4];
double q6 = q[5];

double qs5 = qs[4];
double qs6 = qs[5];

double t2 = cos(qs6);
double t3 = sin(qs5);
double t4 = cos(q2);
double t5 = cos(q1);
double t6 = cos(qs5);
double t7 = t5*t6;
double t8 = sin(q1);
double t9 = t2*t3*t8;
double t10 = t7+t9;
double t11 = t4*t10;
double t12 = sin(q2);
double t13 = sin(qs6);
double t14 = t3*t12*t13;
double t15 = t11+t14;
double t16 = sin(q5);
double t17 = cos(q4);
double t18 = sin(q4);
double t19 = q3-1.56E-2;
double t20 = cos(q5);
double t21 = t2*t12;
double t32 = t4*t8*t13;
double t22 = t21-t32;
double t23 = t6*t8;
double t40 = t2*t3*t5;
double t24 = t23-t40;
double t25 = t2*t4;
double t26 = t8*t12*t13;
double t27 = t25+t26;
double t28 = t17*t27;
double t29 = t5*t13*t18;
double t30 = t28+t29;
double t31 = t16*t30*9.1E-3;
double t33 = t19*t22;
double t34 = t20*t22*9.1E-3;
double t35 = t31+t33+t34;
double t36 = t15*t19;
double t37 = t15*t20*9.1E-3;
double t38 = t10*t12;
double t44 = t3*t4*t13;
double t39 = t38-t44;
double t41 = t18*t24;
double t45 = t17*t39;
double t75 = t41-t45;
double t42 = t16*t75*9.1E-3;
double t43 = t36+t37+t42;
double t46 = t37+t42;
double t47 = t31+t34;
double t48 = t3*t5;
double t52 = t2*t6*t8;
double t49 = t48-t52;
double t50 = t6*t12*t13;
double t53 = t4*t49;
double t51 = t50-t53;
double t54 = t3*t8;
double t55 = t2*t5*t6;
double t56 = t54+t55;
double t57 = t19*t51;
double t58 = t20*t51*9.1E-3;
double t59 = t12*t49;
double t60 = t4*t6*t13;
double t61 = t59+t60;
double t62 = t18*t56;
double t63 = t18*t27;
double t71 = t5*t13*t17;
double t64 = t63-t71;
double t69 = t17*t61;
double t65 = t62-t69;
double t66 = t58-t16*t65*9.1E-3;
double t67 = t18*t61;
double t68 = t17*t56;
double t70 = -t21+t32;
double t72 = -t11-t14;
double t73 = t18*t39;
double t74 = t17*t24;
A0(0,0) = t2*t43-t3*t13*t35;
A0(0,1) = -t24*t35+t5*t13*t43;
A0(0,2) = t51;
A0(0,3) = -t15*t47+t22*t46;
A0(0,4) = -t47*(t73+t74)-t46*t64;
A0(1,0) = t13*(t4*t5*-1.56E2+q3*t4*t5*1.0E4+t4*t5*t20*9.1E1+t8*t16*t18*9.1E1-t5*t12*t16*t17*9.1E1)*1.0E-4;
A0(1,1) = t2*t4*1.56E-2-q3*t2*t4-t2*t4*t20*9.1E-3+t8*t12*t13*1.56E-2-q3*t8*t12*t13+t2*t12*t16*t17*9.1E-3-t8*t12*t13*t20*9.1E-3-t4*t8*t13*t16*t17*9.1E-3;
A0(1,2) = t70;
A0(1,3) = t16*(t2*t4*t18-t5*t13*t17+t8*t12*t13*t18)*9.1E-3;
A0(1,4) = t2*t12*t16*9.1E-3-t4*t8*t13*t16*9.1E-3-t2*t4*t17*t20*9.1E-3-t5*t13*t18*t20*9.1E-3-t8*t12*t13*t17*t20*9.1E-3;
A0(2,0) = t2*(t57+t58-t16*t65*9.1E-3)-t6*t13*t35;
A0(2,1) = t35*t56+t5*t13*(t57+t58-t16*t65*9.1E-3);
A0(2,2) = t72;
A0(2,3) = t22*t66-t47*t51;
A0(2,4) = t47*(t67+t68)-t64*t66;
A0(3,0) = t6*t13;
A0(3,1) = -t54-t55;
A0(3,3) = t51;
A0(3,4) = -t67-t68;
A0(3,5) = -t16*t51-t20*t65;
A0(4,0) = -t2;
A0(4,1) = -t5*t13;
A0(4,3) = t70;
A0(4,4) = t64;
A0(4,5) = t16*t22-t20*t30;
A0(5,0) = -t3*t13;
A0(5,1) = -t23+t40;
A0(5,3) = t72;
A0(5,4) = -t73-t74;
A0(5,5) = t15*t16-t20*t75;

return A0;
}



