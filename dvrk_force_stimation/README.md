# dvrk_dynamics_force_estimation
This package has code related to the daVinci arms force estimation.
Inside the package there is the library containing all the function related to the dynamics and kinematics of the DVRK arms. 

# Install
Copy the package in the dvrk-ros folder and compile with catkin build
If you have any problems with the library it's possible to recompile that. The code is in the dvrk_dynamics folder. Hence copy the build_lib and include folder into the lib folder of the package.

# Contents
Inside the lib folder there are two library libMTM_dynamics.so and libPSM_dynamics.so containing respectively all the symbolic and optimized function for the MTM and the PSM dynamic model. 
The functions names respects the nomenclature reported in the book [1]. Hence: B is the inertia batrix, C is the coriolis and centrifugal matrix, G is the gravity vector, F is the friction matrix, K is the elasticity matrix, J is the jacobian matrix, Te is the direct kinematics matrix. 
It's possible to create a PSM or a MTM arm using the class constructor.
The external forces is calculated using the residual based approach [3]. 

# The dvrk_dynamics Library

```C++
X_dynamics A(arm_name, parameter_path); 
(X=MTM or X=PSM) as it is reported in the example program example.cpp.
```

```C++
Matrix6d PSM_dynamics::PSM_B(Vector7d q) 
calculate the 6X6 PSM inertia matrix. It requires the input 7X1 vector q of the joint position.
```

```C++
Matrix6d PSM_dynamics::PSM_C(Vector7d q, Vector7d dq) 
calculate the 6X6 PSM coriolis and centrifugal matrix. It requires the input 7X1 vector q of the joint position and the 7X1 vector dq of the joint velocities.
```

```C++
Vector6d PSM_dynamics::PSM_G(Vector7d q, Vector6d qs)
calculate the 6X1 PSM gravity vector. It requires the input 7sX1 vector q of the joint position and the 6X1 vector of the setup joint positions. 
More in details the qs5 and qs6 joints of the setup joint are used to change the gravity effect qhen the setup joints are not to the zero position. 
```

```C++
Vector7d PSM_dynamics::PSM_F(Vector7d dq) 
calculate the 7X1 PSM friction vector. It requires the input 7X1 vector dq of the joint velocities. The friction is calculated as a sum of viscous and static friction taking into account also the coupling due to the tendon driving mechanism.
The static frictoion is obtained using a continue function based on the hyperbolic tangent tanh(.) [2]. 
```

```C++
Vector6d PSM_dynamics::PSM_K(Vector7d q) 
calculate the 6X1 PSM elasticity vector. It requires the input 7X1 vector d of the joint positions [2].
```

```C++
Matrix6d PSM_dynamics::PSM_J(Vector7d q, Vector6d qs) 
calculate the 6X6 PSM geometric Jacobian matrix. It requires the input 7X1 vector d of the joint positions and the 6X1 vector of the setup joint positions.
```

```C++
Matrix4d PSM_dynamics::PSM_Te(Vector7d q, Vector6d qs) 
calculate the 4X4 homogeneous transformation matrix for the PSM dyrect kinematics. It requires the input 7X1 vector d of the joint positions and the 6X1 vector of the setup joint positions.
```
---
References:

	- [1] B. Siciliano, L. Villani, Sciavlìicco, "Robotics: Modelling, Planning and Control", 2010, Springer
	- [2] G.A. Fontanelli, F. Ficuciello, L. Villani, B. Siciliano, "Modelling and identification of the da Vinci research kit robotic arms", IEEE/RSJ Int. Conference 
	- [3] A. de Luca and R. Mattone, "Sensorless robot collision detection and hybrid force/motion control", ICRA, 2005 
---
