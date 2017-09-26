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

#include "dvrk_sensing/MTM_dynamics.h"

//******************************************************************************
//Dirkin in base frame
Matrix<3,3> MTM_dynamics::MTM_dJ3(Vector<7> q, Vector<7> dq){

Matrix<3,3> A0 = Zeros;

double q1 = q[0];
double q2 = q[1];
double q3 = q[2];
double q4 = q[3];
double q5 = q[4];
double q6 = q[5];
double q7 = q[6];

double dq1 = dq[0];
double dq2 = dq[1];
double dq3 = dq[2];
double dq4 = dq[3];
double dq5 = dq[4];
double dq6 = dq[5];
double dq7 = dq[6];

  double t2 = q2+q3;
  double t3 = cos(q1);
  double t4 = cos(t2);
  double t5 = t4*1.506E3;
  double t6 = sin(t2);
  double t7 = t6*3.645E3;
  double t8 = sin(q1);
  double t9 = cos(q2);
  double t10 = t4*3.645E-1;
  double t11 = t6*1.506E-1;
  double t12 = sin(q2);
  double t13 = t4*1.506E-1;
  double t14 = t6*3.645E-1;
  double t15 = t10-t11;
  double t16 = t5+t7;
  double t17 = t4*3.645E3;
  double t18 = t12*2.794E3;
  double t19 = t6*-1.506E3+t17+t18;
  double t20 = t5+t7-t9*2.794E3;
  double t26 = t9*2.794E-1;
  double t21 = t13+t14-t26;
  double t22 = t12*2.794E-1;
  double t23 = t10-t11+t22;
  double t24 = dq3*t3*t15;
  double t25 = t13+t14;
  A0[0][0] = dq3*t3*t16*(-1.0E-4)-dq2*t3*t20*1.0E-4-dq1*t8*t19*1.0E-4;
  A0[0][1] = -dq1*t3*t21-dq3*t8*t15-dq2*t8*t23;
  A0[0][2] = -dq2*t8*t15-dq3*t8*t15-dq1*t3*t25;
  A0[1][0] = dq1*t3*t19*1.0E-4-dq3*t8*t16*1.0E-4-dq2*t8*t20*1.0E-4;
  A0[1][1] = t24+dq2*t3*t23-dq1*t8*t21;
  A0[1][2] = t24+dq2*t3*t15-dq1*t8*t25;
  A0[2][1] = -dq2*t21-dq3*t25;
  A0[2][2] = (t4*5.02E2+t6*1.215E3)*(dq2+dq3)*(-3.0E-4);
  return A0;
}
