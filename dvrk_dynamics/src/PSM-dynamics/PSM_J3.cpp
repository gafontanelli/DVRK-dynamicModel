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
Matrix<6,3> PSM_dynamics::PSM_J3(Vector<6> q){

Matrix<6,3> A0 = Zeros;
float q1 = q[0];
float q2 = q[1];
float q3 = q[2];
float q4 = q[3];
float q5 = q[4];
float q6 = q[5];

float qs5 = 0;
float qs6 = 0;

float t2 = cos(qs6);
float t3 = sin(qs5);
float t4 = sin(qs6);
float t5 = sin(q2);
float t6 = cos(q2);
float t7 = sin(q1);
float t8 = q3-1.56E-2;
float t9 = cos(qs5);
float t10 = cos(q1);
float t11 = t2*t5;
float t27 = t4*t6*t7;
float t12 = t11-t27;
float t13 = t9*t10;
float t14 = t2*t3*t7;
float t15 = t13+t14;
float t16 = t6*t15;
float t17 = t3*t4*t5;
float t18 = t16+t17;
float t19 = t3*t10;
float t25 = t2*t7*t9;
float t20 = t19-t25;
float t21 = t4*t5*t9;
float t26 = t6*t20;
float t22 = t21-t26;
float t23 = t7*t9;
float t31 = t2*t3*t10;
float t24 = t23-t31;
float t28 = t3*t7;
float t29 = t2*t9*t10;
float t30 = t28+t29;
A0[0][0] = t2*t8*t18-t3*t4*t8*t12;
A0[0][1] = -t8*t12*t24+t4*t8*t10*t18;
A0[0][2] = t22;
A0[1][0] = -t3*t4*t8*t22+t4*t8*t9*t18;
A0[1][1] = -t8*t22*t24-t8*t18*t30;
A0[1][2] = -t11+t27;
A0[2][0] = t2*t8*(t21-t26)-t4*t8*t9*t12;
A0[2][1] = t8*t12*t30+t4*t8*t10*(t21-t26);
A0[2][2] = -t16-t17;
A0[3][0] = t4*t9;
A0[3][1] = -t28-t29;
A0[4][0] = -t2;
A0[4][1] = -t4*t10;
A0[5][0] = -t3*t4;
A0[5][1] = -t23+t31;
return A0;
}
