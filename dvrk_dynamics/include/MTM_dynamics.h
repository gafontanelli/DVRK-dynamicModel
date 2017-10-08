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
#include <fstream>
#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 7, 7> Matrix7d;
class MTM_dynamics
{
	public:
		string psmName;
		string paramFile;
		double mass1 = 0.0;
		double mpx1  = 0.0;
		double mpy1  = 0.0;
		double mpz1  = 0.0;
		double Ixx1  = 0.0;
		double Iyy1  = 0.0;
		double Izz1  = 0.0;
		double Ixy1  = 0.0;
		double Ixz1  = 0.0;
		double Iyz1  = 0.0;
		double Fv1   = 0.0;
		double Fs1   = 0.0;
		double K1    = 0.0;
		double O1    = 0.0;
		double mass2 = 0.0;
		double mpx2  = 0.0;
		double mpy2  = 0.0;
		double mpz2  = 0.0;
		double Ixx2  = 0.0;
		double Iyy2  = 0.0;
		double Izz2  = 0.0;
		double Ixy2  = 0.0;
		double Fv2   = 0.0;
		double Fs2   = 0.0;
		double mass3 = 0.0;
		double mpx3  = 0.0;
		double mpy3  = 0.0;
		double mpz3  = 0.0;
		double Ixx3  = 0.0;
		double Iyy3  = 0.0;
		double Izz3  = 0.0;
		double mass4 = 0.0;
		double mpx4  = 0.0;
		double mpy4  = 0.0;
		double mpz4  = 0.0;
		double Ixx4  = 0.0;
		double Iyy4  = 0.0;
		double Izz4  = 0.0;
		double mass5 = 0.0;
		double mpx5  = 0.0;
		double mpy5  = 0.0;
		double mpz5  = 0.0;
		double Ixx5  = 0.0;
		double Iyy5  = 0.0;
		double Izz5  = 0.0;
		double Ixz5  = 0.0;
		double Fv3   = 0.0;
		double Fs3   = 0.0;
		double mass6 = 0.0;
		double mpx6  = 0.0;
		double mpy6  = 0.0;
		double mpz6  = 0.0;
		double Ixx6  = 0.0;
		double Iyy6  = 0.0;
		double Izz6  = 0.0;
		double Fv4   = 0.0;
		double Fs4   = 0.0;
		double K4    = 0.0;
		double O4    = 0.0;
		double mass7 = 0.0;
		double mpx7  = 0.0;
		double mpy7  = 0.0;
		double mpz7  = 0.0;
		double Ixx7  = 0.0;
		double Iyy7  = 0.0;
		double Izz7  = 0.0;
		double Fv5   = 0.0;
		double Fs5   = 0.0;
		double K5    = 0.0;
		double O5    = 0.0;
		double mass8 = 0.0;
		double mpx8  = 0.0;
		double mpy8  = 0.0;
		double mpz8  = 0.0;
		double Ixx8  = 0.0;
		double Iyy8  = 0.0;
		double Izz8  = 0.0;
		double Fv6   = 0.0;
		double Fs6   = 0.0;
		double K6    = 0.0;
		double O6    = 0.0;
		double mass9 = 0.0;
		double mpx9  = 0.0;
		double mpy9  = 0.0;
		double mpz9  = 0.0;
		double Ixx9  = 0.0;
		double Iyy9  = 0.0;
		double Izz9  = 0.0;
		double Fv7   = 0.0;
		double Fs7   = 0.0;


		/* brief Constructor. */
		MTM_dynamics(string psmName, string paramFile);
		bool read_parameters_from_file(string paramFile);
		Matrix<double, 6, 7> MTM_J(Vector7d q);	//B
		Matrix4d MTM_Te(Vector7d q);	//B



		Matrix7d MTM_B(Vector7d q);	//B
		Matrix7d MTM_C(Vector7d q, Vector7d dq);
		Vector7d MTM_G(Vector7d q);	//B1
		Vector7d MTM_K(Vector7d q);	//B1
		Vector7d MTM_F(Vector7d dq);	//B1




};

#endif // _MTM_DYNAMICS_H
