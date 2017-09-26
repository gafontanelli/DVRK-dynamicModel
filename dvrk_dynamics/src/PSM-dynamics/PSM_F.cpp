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
Vector<7> PSM_dynamics::PSM_F(Vector<7> dq){

Vector<7> A0 = Zeros;

float dq1 = dq[0];
float dq2 = dq[1];
float dq3 = dq[2];
float dq4 = dq[3];
float dq5 = dq[4];
float dq6 = dq[5];
float dq7 = dq[6];
/*if ( dq1> 0.001 ){ t1 = 1; }else if( dq1<-0.001){ t1 = -1; }else{ t1 = 0; }
if ( dq2> 0.001 ){ t2 = 1; }else if( dq2<-0.001){ t2 = -1; }else{ t2 = 0; }
if ( dq3> 0.001 ){ t3 = 1; }else if( dq3<-0.001){ t3 = -1; }else{ t3 = 0; }
if ( dq4> 0.001 ){ t4 = 1; }else if( dq4<-0.001){ t4 = -1; }else{ t4 = 0; }
if ( dq5> 0.001 ){ t5 = 1; }else if( dq5<-0.001){ t5 = -1; }else{ t5 = 0; }
if ( dq6> 0.001 ){ t6 = 1; }else if( dq6<-0.001){ t6 = -1; }else{ t6 = 0; }
// t1 =0;
// t2 =0;
// t3 =0;
// t4 =0;
// t5 =0;
// t6 =0;

A0[0] = Fv11*dq1+Fs11*(t1);
A0[1] = Fv22*dq2+Fs22*(t2);
A0[2] = Fv33*dq3+Fs33*(t3);
A0[3] = Fv44*dq4+Fs44*(t4);
A0[4] = Fv55*dq5+Fv56*dq6+Fs55*(t5);
A0[5] = Fv65*dq5+Fv66*dq6+Fs66*(t6);//*/


float friction_slope = 50;


float t2 = dq5*6.696E-1;
float t3 = dq6*8.212E-1;
float t4 = dq7*4.106E-1;
float t5 = t2+t3+t4;
float t6 = tanh(friction_slope*t5);//(t5/fabs(t5));
float t7 = t2+t3-t4;
float t8 = tanh(friction_slope*t7);//(t7/fabs(t7));
float t9 = dq7*(-4.106E-1)+t2+t3;
float t10 = tanh(friction_slope*t9);



A0[0] = Fv1*dq1+Fs1*(tanh(friction_slope*dq1));
A0[1] = Fv2*dq2+Fs2*(tanh(friction_slope*dq2));
A0[2] = Fv3*dq3+Fs3*(tanh(friction_slope*dq3));
A0[3] = Fv4*dq4*4.0921609E-1+Fs4*(tanh(friction_slope*dq4))*6.397E-1;
A0[4] = Fv5*dq5*9.6373489E-1+Fv6*dq5*4.4836416E-1+Fv6*dq6*5.4987552E-1+Fv7*dq5*4.4836416E-1-Fv6*dq7*2.7493776E-1+Fv7*dq6*5.4987552E-1+Fv7*dq7*2.7493776E-1+Fs7*t6*6.696E-1+Fs6*(t10)*6.696E-1+Fs5*(tanh(friction_slope*dq5))*9.817E-1;
A0[5] = Fv6*dq5*5.4987552E-1+Fv6*dq6*6.7436944E-1+Fv7*dq5*5.4987552E-1-Fv6*dq7*3.3718472E-1+Fv7*dq6*6.7436944E-1+Fv7*dq7*3.3718472E-1+Fs7*t6*8.212E-1+Fs6*t8*8.212E-1;
A0[6] = Fv6*dq5*(-2.7493776E-1)-Fv6*dq6*3.3718472E-1+Fv7*dq5*2.7493776E-1+Fv6*dq7*1.6859236E-1+Fv7*dq6*3.3718472E-1+Fv7*dq7*1.6859236E-1+Fs7*t6*4.106E-1-Fs6*t8*4.106E-1;

return A0;
}
