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

#include "dvrk_dynamics/PSM_dynamics.h"

//******************************************************************************
//Dirkin in base frame
Matrix<6> PSM_dynamics::PSM_C(Vector<6> q, Vector<6> dq){

Matrix<6> A0 = Zeros;

float q1 = q[0];
float q2 = q[1];
float q3 = q[2];
float q4 = q[3];
float q5 = q[4];
float q6 = q[5];

float dq1 = dq[0];
float dq2 = dq[1];
float dq3 = dq[2];
float dq4 = dq[3];
float dq5 = dq[4];
float dq6 = dq[5];

float qs5 = 0;
float qs6 = 0;

float t2 = q2*2.0;
float t3 = cos(t2);
float t4 = sin(t2);
float t5 = q3*q3;
float t6 = sin(q2);
float t7 = cos(q2);
float t8 = t7*t7;
float t9 = mass9*5.0E2;
float t10 = mpz5*2.5E3;
float t11 = mass5*q3*2.5E3;
float t12 = mass9*q3*2.5E3;
float t13 = mass5*-3.9E1+t9+t10+t11+t12;
A0[0][0] = dq3*mass5*(-1.56E-2)+dq3*mass9*(1.0/5.0)+dq3*mpz5+Ixx2*dq2*t4-Ixx4*dq2*t4-Ixx5*dq2*t4-Ixx9*dq2*t4-Ixz4*dq2*t3*2.0-Ixz5*dq2*t3*2.0-Iyy2*dq2*t4+Izz4*dq2*t4+Izz5*dq2*t4+Izz9*dq2*t4+dq3*mass5*q3+dq3*mass9*q3-dq2*mass3*t4*(1.0/2.5E1)-dq2*mass4*t4*(1.0/2.5E1)-dq2*mass5*t4*2.4336E-4-dq3*mass5*t3*1.56E-2-dq2*mass9*t4*(1.0/2.5E1)+dq3*mass9*t3*(1.0/5.0)-dq2*mpx4*t3*(2.0/5.0)-dq2*mpx5*t3*3.12E-2+dq3*mpx5*t4+dq2*mpy3*t6*(2.0/5.0)+dq2*mpz4*t4*(2.0/5.0)+dq2*mpz5*t4*3.12E-2+dq3*mpz5*t3+dq2*mass5*q3*t4*3.12E-2+dq3*mass5*q3*t3-dq2*mass9*q3*t4*(2.0/5.0)+dq3*mass9*q3*t3+dq2*mpx5*q3*t3*2.0-dq2*mpz5*q3*t4*2.0-dq2*mass5*t4*t5-dq2*mass9*t4*t5;
A0[0][1] = Ixx2*dq1*t4-Ixx4*dq1*t4-Ixx5*dq1*t4-Ixx9*dq1*t4-Ixz4*dq1*t3*2.0-Ixz5*dq1*t3*2.0-Iyy2*dq1*t4+Izz4*dq1*t4+Izz5*dq1*t4+Izz9*dq1*t4-dq1*mass3*t4*(1.0/2.5E1)-dq1*mass4*t4*(1.0/2.5E1)-dq1*mass5*t4*2.4336E-4-dq1*mass9*t4*(1.0/2.5E1)-dq1*mpx4*t3*(2.0/5.0)-dq1*mpx5*t3*3.12E-2+dq1*mpy3*t6*(2.0/5.0)+dq3*mpy5*t6*2.0+dq1*mpz4*t4*(2.0/5.0)+dq1*mpz5*t4*3.12E-2+dq1*mass5*q3*t4*3.12E-2-dq1*mass9*q3*t4*(2.0/5.0)+dq1*mpx5*q3*t3*2.0-dq1*mpz5*q3*t4*2.0-dq1*mass5*t4*t5-dq1*mass9*t4*t5;
A0[0][2] = dq1*mass5*t8*(-3.12E-2)+dq1*mass9*t8*(2.0/5.0)+dq2*mpy5*t6*2.0+dq1*mpz5*t8*2.0+dq1*mass5*q3*t8*2.0+dq1*mass9*q3*t8*2.0+dq1*mpx5*t6*t7*2.0;
A0[1][0] = Ixx2*t4*(-1.0/2.0)+Ixx4*t4*(1.0/2.0)+Ixx5*t4*(1.0/2.0)+Ixx9*t4*(1.0/2.0)+Ixz4*t3+Ixz5*t3+Iyy2*t4*(1.0/2.0)-Izz4*t4*(1.0/2.0)-Izz5*t4*(1.0/2.0)-Izz9*t4*(1.0/2.0)+mass3*t4*(1.0/5.0E1)+mass4*t4*(1.0/5.0E1)+mass5*t4*1.2168E-4+mass9*t4*(1.0/5.0E1)+mpx4*t3*(1.0/5.0)+mpx5*t3*1.56E-2-mpy3*t6*(1.0/5.0)-mpz4*t4*(1.0/5.0)-mpz5*t4*1.56E-2-mass5*q3*t4*1.56E-2+mass9*q3*t4*(1.0/5.0)-mpx5*q3*t3+mpz5*q3*t4+mass5*t4*t5*(1.0/2.0)+mass9*t4*t5*(1.0/2.0);
A0[1][1] = dq3*t13*8.0E-4;
A0[1][2] = dq2*t13*8.0E-4;
A0[2][0] = mass5*t8*1.56E-2-mass9*t8*(1.0/5.0)-mpx5*t4*(1.0/2.0)-mpz5*t8-mass5*q3*t8-mass9*q3*t8;
A0[2][1] = mass5*1.56E-2-mass9*(1.0/5.0)-mpz5-mass5*q3-mass9*q3;


return A0;
}
