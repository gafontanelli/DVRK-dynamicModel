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
Matrix<4,4> PSM_dynamics::PSM_T4(Vector<6> q){

Matrix<4,4> A0 = Zeros;
float q1 = q[0];
float q2 = q[1];
float q3 = q[2];
float q4 = q[3];
float q5 = q[4];
float q6 = q[5];

float qs5 = 0;
float qs6 = 0;

float t2 = cos(qs5);
float t3 = sin(q1);
float t4 = sin(qs5);
float t5 = cos(q1);
float t6 = cos(qs6);
float t7 = sin(q4);
float t8 = sin(q2);
float t9 = t4*t5;
float t20 = t2*t3*t6;
float t10 = t9-t20;
float t11 = t8*t10;
float t12 = cos(q2);
float t13 = sin(qs6);
float t14 = t2*t12*t13;
float t15 = t11+t14;
float t16 = cos(q4);
float t17 = t3*t4;
float t18 = t2*t5*t6;
float t19 = t17+t18;
float t21 = t2*t8*t13;
float t22 = t6*t12;
float t23 = t3*t8*t13;
float t24 = t22+t23;
float t25 = t3*t12*t13;
float t26 = q3-1.56E-2;
float t27 = t2*t5;
float t28 = t3*t4*t6;
float t29 = t27+t28;
float t30 = t8*t29;
float t31 = t30-t4*t12*t13;
float t32 = t2*t3;
float t33 = t32-t4*t5*t6;
A0[0][0] = t7*t19-t15*t16;
A0[0][1] = t7*t15+t16*t19;
A0[0][2] = t21-t10*t12;
A0[0][3] = t26*(t21-t10*t12);
A0[1][0] = t16*t24+t5*t7*t13;
A0[1][1] = -t7*t24+t5*t13*t16;
A0[1][2] = t25-t6*t8;
A0[1][3] = t26*(t25-t6*t8);
A0[2][0] = t7*t33-t16*t31;
A0[2][1] = t7*t31+t16*t33;
A0[2][2] = -t12*t29-t4*t8*t13;
A0[2][3] = -t26*(t12*t29+t4*t8*t13);
A0[3][3] = 1.0;
return A0;
}
