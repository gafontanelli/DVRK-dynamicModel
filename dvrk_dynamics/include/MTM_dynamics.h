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
#ifndef _MTM_DYNAMICS_H
#define _MTM_DYNAMICS_H

#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "MTML_dynamics_parameters.h"
#include <Eigen/Dense>

using namespace Eigen;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 7, 7> Matrix7d;
class MTM_dynamics
{
	public:

		/* brief Constructor. */
		MTM_dynamics(int n);
		
		Matrix<double, 6, 7> MTM_J(Vector7d q);	//B
		Matrix4d MTM_Te(Vector7d q);	//B



		Matrix7d MTM_B(Vector7d q);	//B
		Matrix7d MTM_C(Vector7d q, Vector7d dq);
		Vector7d MTM_G(Vector7d q);	//B1
		Vector7d MTM_K(Vector7d q);	//B1
		Vector7d MTM_F(Vector7d dq);	//B1




};

#endif // _MTM_DYNAMICS_H
