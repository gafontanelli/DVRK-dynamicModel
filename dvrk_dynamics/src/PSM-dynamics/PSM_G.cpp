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
Vector<6> PSM_dynamics::PSM_G(Vector<6> q){

Vector<6> A0 = Zeros;

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
float t4 = cos(q1);
float t5 = cos(qs6);
float t6 = sin(qs5);
float t7 = t2*t4*(9.81E2/1.0E2);
float t8 = t3*t5*t6*(9.81E2/1.0E2);
float t9 = t7+t8;
float t10 = t2*t3;
float t13 = t4*t5*t6;
float t11 = t10-t13;
float t12 = cos(q2);
float t14 = sin(q2);
float t15 = sin(qs6);
float t16 = t6*t14*t15*(9.81E2/1.0E2);
float t17 = t2*t4*t12*(9.81E2/1.0E2);
float t18 = t3*t5*t6*t12*(9.81E2/1.0E2);
float t19 = t16+t17+t18;
float t20 = t2*t4*t14*(9.81E2/5.0E2);
float t21 = t3*t5*t6*t14*(9.81E2/5.0E2);
float t22 = t20+t21-t6*t12*t15*(9.81E2/5.0E2);
float t23 = t2*t4*t14*(9.81E2/1.0E2);
float t24 = t3*t5*t6*t14*(9.81E2/1.0E2);
float t26 = t6*t12*t15*(9.81E2/1.0E2);
float t25 = t23+t24-t26;
float t27 = q3*5.0;
float t28 = t27+1.0;
float t29 = q3*2.5E3;
float t30 = t29-3.9E1;
A0[0] = mpy3*(t2*t3*(9.81E2/1.0E2)-t4*t5*t6*(9.81E2/1.0E2))-mpx1*t11*(9.81E2/1.0E2)+mpy4*t9+mpy5*t9-mpz1*t9-mpz2*t9-mpz3*t9-mass3*t11*t12*(9.81E2/5.0E2)-mass4*t11*t12*(9.81E2/5.0E2)-mpx2*t11*t12*(9.81E2/1.0E2)+mpx4*t11*t14*(9.81E2/1.0E2)+mpx5*t11*t14*(9.81E2/1.0E2)+mpy2*t11*t14*(9.81E2/1.0E2)+mpz4*t11*t12*(9.81E2/1.0E2)+mpz5*t11*t12*(9.81E2/1.0E2)+mass5*t11*t12*t30*3.924E-3-mass9*t11*t12*t28*(9.81E2/5.0E2);
A0[1] = -mass3*t22-mass4*t22-mpx4*t19-mpx5*t19-mpx2*t25-mpy2*t19+mpz4*t25+mpz5*t25-mass9*t28*(t2*t4*t14*9.81E2-t6*t12*t15*9.81E2+t3*t5*t6*t14*9.81E2)*(1.0/5.0E2)+mass5*t30*(t2*t4*t14*4.905E4-t6*t12*t15*4.905E4+t3*t5*t6*t14*4.905E4)*8.0E-8;
A0[2] = (mass5-mass9)*(t2*t4*t12+t6*t14*t15+t3*t5*t6*t12)*(-9.81E2/1.0E2);
return A0;
}
