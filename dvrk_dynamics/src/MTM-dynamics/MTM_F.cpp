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
Vector<7> MTM_dynamics::MTM_F(Vector<7> dq){

Vector<7> A0 = Zeros;
double dq1 = dq[0];
double dq2 = dq[1];
double dq3 = dq[2];
double dq4 = dq[3];
double dq5 = dq[4];
double dq6 = dq[5];
double dq7 = dq[6];

float friction_slope = 50;



A0[0] = Fv1*dq1+Fs1*tanh(friction_slope*dq1);
A0[1] = Fv2*dq2+Fs2*tanh(friction_slope*dq2);
A0[2] = Fv3*dq3+Fs3*tanh(friction_slope*dq3);
A0[3] = Fv4*dq4+Fs4*tanh(friction_slope*dq4);
A0[4] = Fv5*dq5+Fs5*tanh(friction_slope*dq5);
A0[5] = Fv6*dq6+Fs6*tanh(friction_slope*dq6);
A0[6] = Fv7*dq7+Fs7*tanh(friction_slope*dq7);

return A0;
}

