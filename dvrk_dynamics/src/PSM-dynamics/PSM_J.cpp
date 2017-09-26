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
Matrix<6,6> PSM_dynamics::PSM_J(Vector<6> q){

Matrix<6,6> A0 = Zeros;
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
float t4 = cos(q2);
float t5 = cos(q1);
float t6 = cos(qs5);
float t7 = t5*t6;
float t8 = sin(q1);
float t9 = t2*t3*t8;
float t10 = t7+t9;
float t11 = t4*t10;
float t12 = sin(q2);
float t13 = sin(qs6);
float t14 = t3*t12*t13;
float t15 = t11+t14;
float t16 = sin(q5);
float t17 = cos(q4);
float t18 = sin(q4);
float t19 = q3-1.56E-2;
float t20 = cos(q5);
float t21 = t2*t12;
float t32 = t4*t8*t13;
float t22 = t21-t32;
float t23 = t6*t8;
float t40 = t2*t3*t5;
float t24 = t23-t40;
float t25 = t2*t4;
float t26 = t8*t12*t13;
float t27 = t25+t26;
float t28 = t17*t27;
float t29 = t5*t13*t18;
float t30 = t28+t29;
float t31 = t16*t30*9.1E-3;
float t33 = t19*t22;
float t34 = t20*t22*9.1E-3;
float t35 = t31+t33+t34;
float t36 = t15*t19;
float t37 = t15*t20*9.1E-3;
float t38 = t10*t12;
float t44 = t3*t4*t13;
float t39 = t38-t44;
float t41 = t18*t24;
float t45 = t17*t39;
float t75 = t41-t45;
float t42 = t16*t75*9.1E-3;
float t43 = t36+t37+t42;
float t46 = t37+t42;
float t47 = t31+t34;
float t48 = t3*t5;
float t52 = t2*t6*t8;
float t49 = t48-t52;
float t50 = t6*t12*t13;
float t53 = t4*t49;
float t51 = t50-t53;
float t54 = t3*t8;
float t55 = t2*t5*t6;
float t56 = t54+t55;
float t57 = t19*t51;
float t58 = t20*t51*9.1E-3;
float t59 = t12*t49;
float t60 = t4*t6*t13;
float t61 = t59+t60;
float t62 = t18*t56;
float t63 = t18*t27;
float t71 = t5*t13*t17;
float t64 = t63-t71;
float t69 = t17*t61;
float t65 = t62-t69;
float t66 = t58-t16*t65*9.1E-3;
float t67 = t18*t61;
float t68 = t17*t56;
float t70 = -t21+t32;
float t72 = -t11-t14;
float t73 = t18*t39;
float t74 = t17*t24;
A0[0][0] = t2*t43-t3*t13*t35;
A0[0][1] = -t24*t35+t5*t13*t43;
A0[0][2] = t51;
A0[0][3] = -t15*t47+t22*t46;
A0[0][4] = -t47*(t73+t74)-t46*t64;
A0[1][0] = t13*(t4*t5*-1.56E2+q3*t4*t5*1.0E4+t4*t5*t20*9.1E1+t8*t16*t18*9.1E1-t5*t12*t16*t17*9.1E1)*1.0E-4;
A0[1][1] = t2*t4*1.56E-2-q3*t2*t4-t2*t4*t20*9.1E-3+t8*t12*t13*1.56E-2-q3*t8*t12*t13+t2*t12*t16*t17*9.1E-3-t8*t12*t13*t20*9.1E-3-t4*t8*t13*t16*t17*9.1E-3;
A0[1][2] = t70;
A0[1][3] = t16*(t2*t4*t18-t5*t13*t17+t8*t12*t13*t18)*9.1E-3;
A0[1][4] = t2*t12*t16*9.1E-3-t4*t8*t13*t16*9.1E-3-t2*t4*t17*t20*9.1E-3-t5*t13*t18*t20*9.1E-3-t8*t12*t13*t17*t20*9.1E-3;
A0[2][0] = t2*(t57+t58-t16*t65*9.1E-3)-t6*t13*t35;
A0[2][1] = t35*t56+t5*t13*(t57+t58-t16*t65*9.1E-3);
A0[2][2] = t72;
A0[2][3] = t22*t66-t47*t51;
A0[2][4] = t47*(t67+t68)-t64*t66;
A0[3][0] = t6*t13;
A0[3][1] = -t54-t55;
A0[3][3] = t51;
A0[3][4] = -t67-t68;
A0[3][5] = -t16*t51-t20*t65;
A0[4][0] = -t2;
A0[4][1] = -t5*t13;
A0[4][3] = t70;
A0[4][4] = t64;
A0[4][5] = t16*t22-t20*t30;
A0[5][0] = -t3*t13;
A0[5][1] = -t23+t40;
A0[5][3] = t72;
A0[5][4] = -t73-t74;
A0[5][5] = t15*t16-t20*t75;

return A0;
}
