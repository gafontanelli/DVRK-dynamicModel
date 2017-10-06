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
Vector<7> MTM_dynamics::MTM_G(Vector<7> q){

Vector<7> A0 = Zeros;
double q1 = q[0];
double q2 = q[1];
double q3 = q[2];
double q4 = q[3];
double q5 = q[4];
double q6 = q[5];
double q7 = q[6];

double m1 = mass1;
double m2 = mass2;
double m3 = mass3;
double m4 = mass4;
double m5 = mass5;
double m6 = mass6;
double m7 = mass7;
double m8 = mass8;
double m9 = mass9;

double  t2 = cos(q2);
double  t3 = sin(q2);
double  t4 = cos(q3);
double  t5 = sin(q3);
double  t6 = cos(q5);
double  t7 = sin(q4);
double  t8 = cos(q4);
double  t9 = sin(q5);
double  t10 = cos(q7);
double  t11 = sin(q7);
double  t12 = cos(q6);
double  t13 = sin(q6);
double  t14 = m4*t2*t4*(9.81E2/1.0E3);
double  t15 = m6*t2*t4*3.575745;
double  t16 = m7*t2*t4*3.575745;
double  t17 = m8*t2*t4*3.575745;
double  t18 = m9*t2*t4*3.575745;
double  t19 = mpx3*t2*t4*(9.81E2/1.0E2);
double  t20 = mpx5*t2*t4*(9.81E2/1.0E2);
double  t21 = mpx6*t2*t4*t8*(9.81E2/1.0E2);
double  t22 = mpz6*t2*t4*t7*(9.81E2/1.0E2);
double  t23 = mpy7*t3*t5*t7*(9.81E2/1.0E2);
double  t24 = mpx7*t2*t4*t6*t8*(9.81E2/1.0E2);
double  t25 = mpx8*t2*t4*t7*t12*(9.81E2/1.0E2);
double  t26 = mpz8*t2*t4*t7*t13*(9.81E2/1.0E2);
double  t27 = mpz8*t2*t5*t9*t12*(9.81E2/1.0E2);
double  t28 = mpz8*t3*t4*t9*t12*(9.81E2/1.0E2);
double  t29 = mpz9*t2*t4*t7*t13*(9.81E2/1.0E2);
double  t30 = mpz9*t2*t5*t9*t12*(9.81E2/1.0E2);
double  t31 = mpz9*t3*t4*t9*t12*(9.81E2/1.0E2);
double  t32 = mpy8*t3*t5*t8*t9*(9.81E2/1.0E2);
double  t33 = mpz7*t3*t5*t8*t9*(9.81E2/1.0E2);
double  t34 = mpx8*t2*t4*t6*t8*t13*(9.81E2/1.0E2);
double  t35 = mpx9*t2*t4*t7*t10*t12*(9.81E2/1.0E2);
double  t36 = mpz8*t3*t5*t6*t8*t12*(9.81E2/1.0E2);
double  t37 = mpz9*t3*t5*t6*t8*t12*(9.81E2/1.0E2);
double  t38 = mpy9*t3*t5*t8*t9*t10*(9.81E2/1.0E2);
double  t39 = mpx9*t3*t5*t8*t9*t11*(9.81E2/1.0E2);
double  t40 = mpy9*t3*t5*t7*t11*t12*(9.81E2/1.0E2);
double  t41 = mpy9*t2*t5*t9*t11*t13*(9.81E2/1.0E2);
double  t42 = mpy9*t3*t4*t9*t11*t13*(9.81E2/1.0E2);
double  t43 = mpy9*t3*t5*t6*t8*t11*t13*(9.81E2/1.0E2);
double  t44 = mpx9*t2*t4*t6*t8*t10*t13*(9.81E2/1.0E2);
double  t45 = q2+q3;
double  t46 = cos(t45);
double  t47 = sin(t45);
double  t48 = t6*t46*(9.81E2/1.0E2);
double  t49 = t48-t8*t9*t47*(9.81E2/1.0E2);
double  t50 = t9*t46*(9.81E2/1.0E2);
double  t51 = t6*t8*t47*(9.81E2/1.0E2);
double  t52 = t50+t51;
double  t53 = t13*t52;
double  t54 = t7*t12*t47*(9.81E2/1.0E2);
double  t55 = t53+t54;
A0[1] = t14+t15+t16+t17+t18+t19+t20+t21+t22+t23+t24+t25+t26+t27+t28+t29+t30+t31+t32+t33+t34+t35+t36+t37+t38+t39+t40+t41+t42+t43+t44+m5*t3*2.740914+m6*t3*2.740914+m7*t3*2.740914+m8*t3*2.740914+m9*t3*2.740914+mpx2*t3*(9.81E2/1.0E2)+mpx4*t3*(9.81E2/1.0E2)+mpy2*t2*(9.81E2/1.0E2)+mpy4*t2*(9.81E2/1.0E2)-m4*t3*t5*(9.81E2/1.0E3)-m6*t3*t5*3.575745-m7*t2*t5*1.477386-m7*t3*t4*1.477386-m7*t3*t5*3.575745-m8*t2*t5*1.477386-m8*t3*t4*1.477386-m8*t3*t5*3.575745-m9*t2*t5*1.477386-m9*t3*t4*1.477386-m9*t3*t5*3.575745-mpx3*t3*t5*(9.81E2/1.0E2)-mpx5*t3*t5*(9.81E2/1.0E2)-mpy3*t2*t5*(9.81E2/1.0E2)-mpy3*t3*t4*(9.81E2/1.0E2)-mpy6*t2*t5*(9.81E2/1.0E2)-mpy6*t3*t4*(9.81E2/1.0E2)-mpz5*t2*t5*(9.81E2/1.0E2)-mpz5*t3*t4*(9.81E2/1.0E2)-mpx6*t3*t5*t8*(9.81E2/1.0E2)-mpx7*t2*t5*t9*(9.81E2/1.0E2)-mpx7*t3*t4*t9*(9.81E2/1.0E2)-mpy7*t2*t4*t7*(9.81E2/1.0E2)-mpy8*t2*t5*t6*(9.81E2/1.0E2)-mpy8*t3*t4*t6*(9.81E2/1.0E2)-mpz7*t2*t5*t6*(9.81E2/1.0E2)-mpz7*t3*t4*t6*(9.81E2/1.0E2)-mpz6*t3*t5*t7*(9.81E2/1.0E2)-mpx7*t3*t5*t6*t8*(9.81E2/1.0E2)-mpx9*t2*t5*t6*t11*(9.81E2/1.0E2)-mpx9*t3*t4*t6*t11*(9.81E2/1.0E2)-mpx8*t3*t5*t7*t12*(9.81E2/1.0E2)-mpx8*t2*t5*t9*t13*(9.81E2/1.0E2)-mpx8*t3*t4*t9*t13*(9.81E2/1.0E2)-mpy8*t2*t4*t8*t9*(9.81E2/1.0E2)-mpy9*t2*t5*t6*t10*(9.81E2/1.0E2)-mpy9*t3*t4*t6*t10*(9.81E2/1.0E2)-mpz7*t2*t4*t8*t9*(9.81E2/1.0E2)-mpz8*t3*t5*t7*t13*(9.81E2/1.0E2)-mpz9*t3*t5*t7*t13*(9.81E2/1.0E2)-mpx8*t3*t5*t6*t8*t13*(9.81E2/1.0E2)-mpx9*t2*t4*t8*t9*t11*(9.81E2/1.0E2)-mpx9*t3*t5*t7*t10*t12*(9.81E2/1.0E2)-mpx9*t2*t5*t9*t10*t13*(9.81E2/1.0E2)-mpx9*t3*t4*t9*t10*t13*(9.81E2/1.0E2)-mpy9*t2*t4*t8*t9*t10*(9.81E2/1.0E2)-mpy9*t2*t4*t7*t11*t12*(9.81E2/1.0E2)-mpz8*t2*t4*t6*t8*t12*(9.81E2/1.0E2)-mpz9*t2*t4*t6*t8*t12*(9.81E2/1.0E2)-mpx9*t3*t5*t6*t8*t10*t13*(9.81E2/1.0E2)-mpy9*t2*t4*t6*t8*t11*t13*(9.81E2/1.0E2);
A0[2] = t14+t15+t16+t17+t18+t19+t20+t21+t22+t23+t24+t25+t26+t27+t28+t29+t30+t31+t32+t33+t34+t35+t36+t37+t38+t39+t40+t41+t42+t43+t44-m4*t3*t5*(9.81E2/1.0E3)-m6*t3*t5*3.575745-m7*t2*t5*1.477386-m7*t3*t4*1.477386-m7*t3*t5*3.575745-m8*t2*t5*1.477386-m8*t3*t4*1.477386-m8*t3*t5*3.575745-m9*t2*t5*1.477386-m9*t3*t4*1.477386-m9*t3*t5*3.575745-mpx3*t3*t5*(9.81E2/1.0E2)-mpx5*t3*t5*(9.81E2/1.0E2)-mpy3*t2*t5*(9.81E2/1.0E2)-mpy3*t3*t4*(9.81E2/1.0E2)-mpy6*t2*t5*(9.81E2/1.0E2)-mpy6*t3*t4*(9.81E2/1.0E2)-mpz5*t2*t5*(9.81E2/1.0E2)-mpz5*t3*t4*(9.81E2/1.0E2)-mpx6*t3*t5*t8*(9.81E2/1.0E2)-mpx7*t2*t5*t9*(9.81E2/1.0E2)-mpx7*t3*t4*t9*(9.81E2/1.0E2)-mpy7*t2*t4*t7*(9.81E2/1.0E2)-mpy8*t2*t5*t6*(9.81E2/1.0E2)-mpy8*t3*t4*t6*(9.81E2/1.0E2)-mpz7*t2*t5*t6*(9.81E2/1.0E2)-mpz7*t3*t4*t6*(9.81E2/1.0E2)-mpz6*t3*t5*t7*(9.81E2/1.0E2)-mpx7*t3*t5*t6*t8*(9.81E2/1.0E2)-mpx9*t2*t5*t6*t11*(9.81E2/1.0E2)-mpx9*t3*t4*t6*t11*(9.81E2/1.0E2)-mpx8*t3*t5*t7*t12*(9.81E2/1.0E2)-mpx8*t2*t5*t9*t13*(9.81E2/1.0E2)-mpx8*t3*t4*t9*t13*(9.81E2/1.0E2)-mpy8*t2*t4*t8*t9*(9.81E2/1.0E2)-mpy9*t2*t5*t6*t10*(9.81E2/1.0E2)-mpy9*t3*t4*t6*t10*(9.81E2/1.0E2)-mpz7*t2*t4*t8*t9*(9.81E2/1.0E2)-mpz8*t3*t5*t7*t13*(9.81E2/1.0E2)-mpz9*t3*t5*t7*t13*(9.81E2/1.0E2)-mpx8*t3*t5*t6*t8*t13*(9.81E2/1.0E2)-mpx9*t2*t4*t8*t9*t11*(9.81E2/1.0E2)-mpx9*t3*t5*t7*t10*t12*(9.81E2/1.0E2)-mpx9*t2*t5*t9*t10*t13*(9.81E2/1.0E2)-mpx9*t3*t4*t9*t10*t13*(9.81E2/1.0E2)-mpy9*t2*t4*t8*t9*t10*(9.81E2/1.0E2)-mpy9*t2*t4*t7*t11*t12*(9.81E2/1.0E2)-mpz8*t2*t4*t6*t8*t12*(9.81E2/1.0E2)-mpz9*t2*t4*t6*t8*t12*(9.81E2/1.0E2)-mpx9*t3*t5*t6*t8*t10*t13*(9.81E2/1.0E2)-mpy9*t2*t4*t6*t8*t11*t13*(9.81E2/1.0E2);
A0[3] = t47*(-mpx6*t7-mpy7*t8+mpz6*t8-mpx7*t6*t7+mpx8*t8*t12+mpy8*t7*t9+mpz7*t7*t9+mpz8*t8*t13+mpz9*t8*t13-mpx8*t6*t7*t13+mpx9*t7*t9*t11+mpx9*t8*t10*t12+mpy9*t7*t9*t10-mpy9*t8*t11*t12+mpz8*t6*t7*t12+mpz9*t6*t7*t12-mpx9*t6*t7*t10*t13+mpy9*t6*t7*t11*t13)*(9.81E2/1.0E2);
A0[4] = mpx7*t6*t46*(9.81E2/1.0E2)-mpy8*t9*t46*(9.81E2/1.0E2)-mpz7*t9*t46*(9.81E2/1.0E2)-mpx7*t8*t9*t47*(9.81E2/1.0E2)+mpx8*t6*t13*t46*(9.81E2/1.0E2)-mpx9*t9*t11*t46*(9.81E2/1.0E2)-mpy8*t6*t8*t47*(9.81E2/1.0E2)-mpy9*t9*t10*t46*(9.81E2/1.0E2)-mpz7*t6*t8*t47*(9.81E2/1.0E2)-mpz8*t6*t12*t46*(9.81E2/1.0E2)-mpz9*t6*t12*t46*(9.81E2/1.0E2)-mpx9*t6*t8*t11*t47*(9.81E2/1.0E2)+mpx9*t6*t10*t13*t46*(9.81E2/1.0E2)-mpx8*t8*t9*t13*t47*(9.81E2/1.0E2)-mpy9*t6*t8*t10*t47*(9.81E2/1.0E2)-mpy9*t6*t11*t13*t46*(9.81E2/1.0E2)+mpz8*t8*t9*t12*t47*(9.81E2/1.0E2)+mpz9*t8*t9*t12*t47*(9.81E2/1.0E2)-mpx9*t8*t9*t10*t13*t47*(9.81E2/1.0E2)+mpy9*t8*t9*t11*t13*t47*(9.81E2/1.0E2);
A0[5] = mpx8*t7*t13*t47*(-9.81E2/1.0E2)+mpx8*t9*t12*t46*(9.81E2/1.0E2)+mpz8*t7*t12*t47*(9.81E2/1.0E2)+mpz9*t7*t12*t47*(9.81E2/1.0E2)+mpz8*t9*t13*t46*(9.81E2/1.0E2)+mpz9*t9*t13*t46*(9.81E2/1.0E2)+mpx8*t6*t8*t12*t47*(9.81E2/1.0E2)-mpx9*t7*t10*t13*t47*(9.81E2/1.0E2)+mpx9*t9*t10*t12*t46*(9.81E2/1.0E2)+mpy9*t7*t11*t13*t47*(9.81E2/1.0E2)-mpy9*t9*t11*t12*t46*(9.81E2/1.0E2)+mpz8*t6*t8*t13*t47*(9.81E2/1.0E2)+mpz9*t6*t8*t13*t47*(9.81E2/1.0E2)+mpx9*t6*t8*t10*t12*t47*(9.81E2/1.0E2)-mpy9*t6*t8*t11*t12*t47*(9.81E2/1.0E2);
A0[6] = mpx9*(t10*t49-t11*t55)-mpy9*(t11*t49+t10*t55);

return A0;
}
