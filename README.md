# dvrk dynamics
This package has code related to daVinci arms dynamic model. 
The dvrk_dynamics is a ros package containing the function for the dynamics, kinematics and for the external force reconstruction.

# Install
Copy the package in the dvrk-ros folder and compile with catkin build

# Contents
The folders MTM-dynamics and PSM-dynamics contains respectively all the symbolic and optimized function for the MTM and the PSM dynamic model. 
The functions names respects the nomenclature reported in the book:
---
references:
- id: fenner2012a
  title: One-click science marketing
  author:
  - family: Fenner
    given: Martin
  container-title: Nature Materials
  volume: 11
  URL: 'http://dx.doi.org/10.1038/nmat3283'
  DOI: 10.1038/nmat3283
  issue: 4
  publisher: Nature Publishing Group
  page: 261-263
  type: article-journal
  issued:
    year: 2012
    month: 3
---

B. Siciliano, L. Villani, Sciavlìicco
Robotics: Modelling, Planning and Control 2010 Springer