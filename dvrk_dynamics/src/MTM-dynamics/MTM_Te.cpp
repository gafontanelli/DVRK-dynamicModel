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
Matrix<4,4> MTM_dynamics::MTM_Te(Vector<7> q){

Matrix<4,4> A0 = Zeros;
double q1 = q[0];
double q2 = q[1];
double q3 = q[2];
double q4 = q[3];
double q5 = q[4];
double q6 = q[5];
double q7 = q[6];

double  t2 = sin(q1);
double  t3 = cos(q1);
double  t4 = sin(q4);
double  t5 = cos(q4);
double  t6 = sin(q2);
double  t7 = sin(q3);
double  t8 = cos(q2);
double  t9 = cos(q3);
double  t13 = t2*t6*t7;
double  t14 = t2*t8*t9;
double  t10 = t13-t14;
double  t11 = sin(q5);
double  t12 = t3*t4;
double  t27 = t5*t10;
double  t15 = t12-t27;
double  t16 = cos(q5);
double  t17 = t2*t7*t8;
double  t18 = t2*t6*t9;
double  t19 = t17+t18;
double  t20 = sin(q7);
double  t21 = cos(q6);
double  t22 = t3*t5;
double  t23 = t4*t10;
double  t24 = t22+t23;
double  t25 = t21*t24;
double  t26 = sin(q6);
double  t28 = t11*t19;
double  t35 = t15*t16;
double  t29 = t26*(t28-t35);
double  t30 = t25+t29;
double  t31 = cos(q7);
double  t32 = t11*t15;
double  t33 = t16*t19;
double  t34 = t32+t33;
double  t36 = q2+q3;
double  t37 = t3*t6*t7;
double  t40 = t3*t8*t9;
double  t38 = t37-t40;
double  t39 = t2*t4;
double  t41 = t3*t7*t8;
double  t42 = t3*t6*t9;
double  t43 = t41+t42;
double  t44 = t5*t38;
double  t45 = t39+t44;
double  t46 = t2*t5;
double  t55 = t4*t38;
double  t47 = t46-t55;
double  t48 = t21*t47;
double  t49 = t11*t43;
double  t50 = t16*t45;
double  t51 = t49+t50;
double  t52 = t48-t26*t51;
double  t53 = t11*t45;
double  t54 = t53-t16*t43;
double  t56 = cos(t36);
double  t57 = t56*3.645E3;
double  t58 = sin(t36);
double  t59 = t6*2.794E3;
double  t60 = t57-t58*1.506E3+t59;
double  t61 = t16*t56;
double  t62 = t61-t5*t11*t58;
double  t63 = t4*t21*t58;
double  t64 = t11*t26*t56;
double  t65 = t5*t16*t26*t58;
double  t66 = t63+t64+t65;
A0[0][0] = -t20*t34-t30*t31;
A0[0][1] = t20*t30-t31*t34;
A0[0][2] = -t24*t26+t21*(t28-t35);
A0[0][3] = t2*t60*1.0E-4;
A0[1][0] = -t20*t54-t31*t52;
A0[1][1] = t20*t52-t31*t54;
A0[1][2] = -t21*t51-t26*t47;
A0[1][3] = t3*t60*(-1.0E-4);
A0[2][0] = t20*t62+t31*t66;
A0[2][1] = -t20*t66+t31*t62;
A0[2][2] = -t21*(t11*t56+t5*t16*t58)+t4*t26*t58;
A0[2][3] = t8*(-2.794E-1)+t56*1.506E-1+t58*3.645E-1;
A0[3][3] = 1.0;

return A0;
}