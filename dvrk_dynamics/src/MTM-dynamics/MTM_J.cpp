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

#include "dvrk_dynamics/MTM_dynamics.h"

//******************************************************************************
//Dirkin in base frame
Matrix<6,7> MTM_dynamics::MTM_J(Vector<7> q){

Matrix<6,7> A0 = Zeros;
double q1 = q[0];
double q2 = q[1];
double q3 = q[2];
double q4 = q[3];
double q5 = q[4];
double q6 = q[5];
double q7 = q[6];

double  t2 = q2+q3;
double  t3 = cos(t2);
double  t4 = sin(t2);
double  t5 = sin(q1);
double  t6 = t3*1.506E-1;
double  t7 = t4*3.645E-1;
double  t8 = t3*3.645E3;
double  t9 = sin(q2);
double  t10 = t9*2.794E3;
double  t11 = t4*-1.506E3+t8+t10;
double  t12 = cos(q1);
double  t13 = cos(q2);
double  t14 = t6+t7-t13*2.794E-1;
double  t15 = t6+t7;
double  t16 = t3*3.645E-1;
double  t17 = sin(q4);
double  t18 = cos(q4);
double  t19 = sin(q3);
double  t20 = cos(q3);
double  t22 = t5*t9*t19;
double  t23 = t5*t13*t20;
double  t21 = t22-t23;
double  t24 = cos(q5);
double  t25 = t12*t17;
double  t26 = t25-t18*t21;
double  t27 = sin(q5);
double  t28 = t5*t13*t19;
double  t29 = t5*t9*t20;
double  t30 = t28+t29;
double  t31 = t9*t12*t19;
double  t34 = t12*t13*t20;
double  t32 = t31-t34;
double  t33 = sin(q6);
double  t35 = t17*t32;
double  t36 = cos(q6);
double  t37 = t5*t17;
double  t38 = t12*t13*t19;
double  t39 = t9*t12*t20;
double  t40 = t38+t39;
double  t41 = t18*t32;
double  t42 = t37+t41;

A0[0][0] = t11*t12*1.0E-4;
A0[0][1] = -t5*t14;
A0[0][2] = -t5*t15;
A0[1][0] = t5*t11*1.0E-4;
A0[1][1] = t12*t14;
A0[1][2] = t12*t15;
A0[2][1] = t4*(-1.506E-1)+t9*2.794E-1+t16;
A0[2][2] = t4*(-1.506E-1)+t16;
A0[3][1] = -t12;
A0[3][2] = -t12;
A0[3][3] = -t4*t5;
A0[3][4] = -t12*t18-t17*t21;
A0[3][5] = -t26*t27-t24*t30;
A0[3][6] = -t33*(t12*t18+t17*t21)-t36*(t24*t26-t27*t30);
A0[4][1] = -t5;
A0[4][2] = -t5;
A0[4][3] = t4*t12;
A0[4][4] = t35-t5*t18;
A0[4][5] = t24*t40-t27*t42;
A0[4][6] = -t36*(t24*t42+t27*t40)+t33*(t35-t5*t18);
A0[5][0] = 1.0;
A0[5][3] = t3;
A0[5][4] = t4*t17;
A0[5][5] = t3*t24-t4*t18*t27;
A0[5][6] = -t36*(t3*t27+t4*t18*t24)+t4*t17*t33;

return A0;
}
