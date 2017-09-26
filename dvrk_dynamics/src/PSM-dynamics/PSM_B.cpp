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
Matrix<6,6> PSM_dynamics::PSM_B(Vector<6> q){

Matrix<6,6> A0 = Zeros;
float q1 = q[0];
float q2 = q[1];
float q3 = q[2];
float q4 = q[3];
float q5 = q[4];
float q6 = q[5];
float qs5 = 0;
float qs6 = 0;

float t2 = cos(q2);
float t3 = t2*t2;
float t4 = t3-1.0;
float t5 = q2*2.0;
float t6 = sin(t5);
float t7 = sin(q2);
float t8 = Ixy4*t2;
float t9 = Ixy5*t2;
float t10 = mpz3*t7*(1.0/5.0);
float t11 = q3*2.5E3;
float t12 = t11-3.9E1;
float t13 = q3*5.0;
float t14 = t13+1.0;
A0[0][0] = Ixx3+Iyy1-Ixx2*t4+Ixx4*t3+Ixx5*t3+Ixx9*t3-Ixz4*t6-Ixz5*t6+Iyy2*t3-Izz4*t4-Izz5*t4-Izz9*t4+mass3*t3*(1.0/2.5E1)+mass4*t3*(1.0/2.5E1)-mpy3*t2*(2.0/5.0)-mpz4*t3*(2.0/5.0)-mpz5*(t3*3.12E-2-q3*t3*2.0)-mpx5*(t6*1.56E-2-q3*t6)-mpx4*t2*t7*(2.0/5.0)+mass9*t14*(t3*2.0E1+q3*t3*1.0E2)*(1.0/5.0E2)-mass5*t12*(t3*3.9E1-q3*t3*2.5E3)*1.6E-7;
A0[0][1] = t8+t9+t10-Iyz4*t7-Iyz5*t7-mpy4*t7*(1.0/5.0)-mpy5*(t7*1.56E-2-q3*t7);
A0[0][2] = -mpy5*t2;
A0[1][0] = t8+t9+t10-Iyz4*t7-Iyz5*t7-mpy4*t7*(1.0/5.0)+mpy5*t7*t12*4.0E-4;
A0[1][1] = Iyy4+Iyy5+Iyy9+Izz2+mass3*(1.0/2.5E1)+mass4*(1.0/2.5E1)-mpz4*(2.0/5.0)+mpz5*(q3*2.0-3.12E-2)+mass9*t14*(q3*1.0E2+2.0E1)*(1.0/5.0E2)+mass5*t12*(q3*5.0E3-7.8E1)*8.0E-8;
A0[1][2] = mpx5;
A0[2][0] = -mpy5*t2;
A0[2][1] = mpx5;
A0[2][2] = mass5+mass9;
return A0;
}
