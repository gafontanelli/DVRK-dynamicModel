# dvrk_dynamics_force_estimation
This package has code related to the daVinci arms force estimation.
Inside the package there is the library containing all the function related to the dynamics and kinematics of the DVRK arms. 

# Install
Copy the package in the dvrk-ros folder and compile with catkin build

# Contents
Inside the lib folder there are two library libMTM_dynamics.so and libPSM_dynamics.so containing respectively all the symbolic and optimized function for the MTM and the PSM dynamic model. 
The functions names respects the nomenclature reported in the book [1]. Hence: B is the inertia batrix, C is the coriolis and centrifugal matrix, G is the gravity matrix, F is the friction matrix, K is the elasticity matrix, J is the jacobian matrix, Te is the direct kinematics matrix. 
It's possible to create a PSM or a MTM arm using the class constructor.

```C++
X_dynamics A('arm name', parameter_path); 
```

(X=MTM or X=PSM) as it is reported in the example program example.cpp.
# List of function





---
References:

	- [1] B. Siciliano, L. Villani, Sciavlìicco, "Robotics: Modelling, Planning and Control", 2010, Springer
---