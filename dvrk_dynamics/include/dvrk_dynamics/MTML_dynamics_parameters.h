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

#define   mass1   3.98902317
#define   mpx1   0.0000000018373234
#define   mpy1   8.01868026e-10
#define   mpz1   0.00000000156920887
#define   Ixx1   25771.4553
#define   Iyy1   -18751.5128
#define   Izz1   0.000000197329048
#define   Ixy1   25771.4552
#define   Ixz1   0.000000204341193
#define   Iyz1   12137.9334
#define   Fv1   0.114928004
#define   Fs1   0.0335956068
#define   K1   0.230807636
#define   O1   -0.0269192362
#define   mass2   3.98910168
#define   mpx2   0.00000184021079
#define   mpy2   0.000154563565
#define   mpz2   0.0000066342707
#define   Ixx2   2177.98876
#define   Iyy2   0.00136823699
#define   Izz2   -2.93849079
#define   Ixy2   0.00749099027
#define   Fv2   0.0937196985
#define   Fs2   0.0508778112
#define   mass3   3.98979694
#define   mpx3   -2.67361636
#define   mpy3   0.000872229684
#define   mpz3   -0.30883266
#define   Ixx3   4401.69649
#define   Iyy3   6785.9811
#define   Izz3   -2794.95867
#define   mass4   3.99056353
#define   mpx4   -5.46735284
#define   mpy4   0.0153594919
#define   mpz4   -0.529340322
#define   Ixx4   3869.93719
#define   Iyy4   6046.43759
#define   Izz4   1.50617553
#define   mass5   3.98937004
#define   mpx5   -3.5080844
#define   mpy5   -0.0169628542
#define   mpz5   -0.841891308
#define   Ixx5   4011.26252
#define   Iyy5   -1497.22767
#define   Izz5   5915.58497
#define   Ixz5   0.364288937
#define   Fv3   0.064600011
#define   Fs3   -0.000000903716616
#define   mass6   3.98841374
#define   mpx6   0.00773689939
#define   mpy6   -0.937516962
#define   mpz6   -0.035088645
#define   Ixx6   2123.5935
#define   Iyy6   -2166.2574
#define   Izz6   4289.88857
#define   Fv4   0.0585574814
#define   Fs4   0.0396429717
#define   K4   0.0365640287
#define   O4   0.0419638083
#define   mass7   3.9878234
#define   mpx7   0.00182423136
#define   mpy7   -0.0546098596
#define   mpz7   -0.00249706373
#define   Ixx7   1046.51029
#define   Iyy7   -1119.74478
#define   Izz7   2166.25485
#define   Fv5   0.00130821513
#define   Fs5   0.009322484
#define   K5   -0.0193341169
#define   O5   0.0108708886
#define   mass8   3.98781055
#define   mpx8   -0.0000238831012
#define   mpy8   -0.00186362474
#define   mpz8   0.00213042142
#define   Ixx8   637.945747
#define   Iyy8   -481.799185
#define   Izz8   1119.74653
#define   Fv6   -0.0000106840065
#define   Fs6   0.0053534296
#define   K6   0.00155005159
#define   O6   -0.00183995811
#define   mass9   3.98781052
#define   mpx9   0.0000116708815
#define   mpy9   0.00002132189
#define   mpz9   0.00284952108
#define   Ixx9   481.799495
#define   Iyy9   481.799615
#define   Izz9   0.0000696967741
#define   Fv7   0.000599694963
#define   Fs7   -0.0000337640889

