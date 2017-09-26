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
Matrix<4,4> PSM_dynamics::PSM_Te(Vector<6> q){

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
float t3 = sin(q2);
float t4 = cos(q1);
float t5 = sin(qs5);
float t6 = t4*t5;
float t7 = cos(qs6);
float t8 = sin(q1);
float t13 = t2*t7*t8;
float t9 = t6-t13;
float t10 = cos(q2);
float t11 = sin(qs6);
float t12 = sin(q4);
float t14 = t3*t9;
float t15 = t2*t10*t11;
float t16 = t14+t15;
float t17 = cos(q4);
float t18 = t5*t8;
float t19 = t2*t4*t7;
float t20 = t18+t19;
float t21 = sin(q5);
float t22 = t9*t10;
float t27 = t2*t3*t11;
float t23 = t22-t27;
float t24 = cos(q5);
float t25 = t12*t20;
float t26 = cos(q6);
float t28 = t23*t24;
float t35 = t16*t17;
float t36 = t25-t35;
float t29 = t21*t36;
float t30 = t28+t29;
float t31 = sin(q6);
float t32 = t12*t16;
float t33 = t17*t20;
float t34 = t32+t33;
float t37 = t7*t10;
float t38 = t3*t8*t11;
float t39 = t37+t38;
float t40 = t17*t39;
float t41 = t4*t11*t12;
float t42 = t40+t41;
float t43 = t3*t7;
float t48 = t8*t10*t11;
float t44 = t43-t48;
float t45 = t12*t39;
float t46 = t45-t4*t11*t17;
float t47 = t21*t42;
float t49 = t24*t44;
float t50 = t47+t49;
float t51 = q3-1.56E-2;
float t52 = t2*t4;
float t53 = t5*t7*t8;
float t54 = t52+t53;
float t55 = t3*t54;
float t62 = t5*t10*t11;
float t56 = t55-t62;
float t57 = t2*t8;
float t63 = t4*t5*t7;
float t58 = t57-t63;
float t59 = t10*t54;
float t60 = t3*t5*t11;
float t61 = t59+t60;
float t64 = t12*t58;
float t65 = t24*t61;
float t71 = t17*t56;
float t72 = t64-t71;
float t66 = t21*t72;
float t67 = t65+t66;
float t68 = t12*t56;
float t69 = t17*t58;
float t70 = t68+t69;
A0[0][0] = -t26*t34-t30*t31;
A0[0][1] = -t21*t23+t24*(t25-t16*t17);
A0[0][2] = -t26*t30+t31*t34;
A0[0][3] = t23*t24*(-9.1E-3)-t21*t36*9.1E-3-t23*t51;
A0[1][0] = t26*t46-t31*t50;
A0[1][1] = -t21*t44+t24*t42;
A0[1][2] = -t26*t50-t31*t46;
A0[1][3] = t21*t42*(-9.1E-3)-t24*t44*9.1E-3-t44*t51;
A0[2][0] = -t26*t70-t31*t67;
A0[2][1] = -t21*t61+t24*(t64-t17*t56);
A0[2][2] = -t26*t67+t31*t70;
A0[2][3] = t24*t61*(-9.1E-3)-t21*t72*9.1E-3-t51*t61;
A0[3][3] = 1.0;
return A0;
}
