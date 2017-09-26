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
Matrix<4,4> PSM_dynamics::PSM_T3(Vector<6> q){

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
float t7 = cos(q2);
float t8 = t4*t5;
float t12 = t2*t3*t6;
float t9 = t8-t12;
float t10 = sin(q2);
float t11 = sin(qs6);
float t13 = t2*t10*t11;
float t14 = t3*t7*t11;
float t15 = q3-1.56E-2;
float t16 = t2*t5;
float t17 = t3*t4*t6;
float t18 = t16+t17;
A0[0][0] = -t9*t10-t2*t7*t11;
A0[0][1] = t3*t4+t2*t5*t6;
A0[0][2] = t13-t7*t9;
A0[0][3] = t15*(t13-t7*t9);
A0[1][0] = t6*t7+t3*t10*t11;
A0[1][1] = t5*t11;
A0[1][2] = t14-t6*t10;
A0[1][3] = t15*(t14-t6*t10);
A0[2][0] = -t10*t18+t4*t7*t11;
A0[2][1] = t2*t3-t4*t5*t6;
A0[2][2] = -t7*t18-t4*t10*t11;
A0[2][3] = -t15*(t7*t18+t4*t10*t11);
A0[3][3] = 1.0;
return A0;
}
