# dvrk dynamics
This package has code related to the daVinci arms dynamic model. 
The dvrk_dynamics is a ros package containing the function for the dynamics, kinematics and for the external force reconstruction.

# Install
Copy the package in the dvrk-ros folder and compile with catkin build

# Contents
The folders MTM-dynamics and PSM-dynamics contains respectively all the symbolic and optimized function for the MTM and the PSM dynamic model. 
The functions names respects the nomenclature reported in the book [1]. Hence: B is the inertia batrix, C is the coriolis and centrifugal matrix, G is the gravity matrix, F is the friction matrix, K is the elasticity matrix, J is the jacobian matrix, Te is the direct kinematics matrix.



---
References:

	- [1] B. Siciliano, L. Villani, Sciavlìicco, "Robotics: Modelling, Planning and Control", 2010, Springer
---