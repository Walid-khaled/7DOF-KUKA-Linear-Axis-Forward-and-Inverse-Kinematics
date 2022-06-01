## KUKA-Linear-Axis-Forward-and-Inverse-Kinematics
Kuka on linear axis includes a further axis to the robot, so it is considerably extending the workspace of the robot. The advantages of the redundant robots are increasing manipulability in specified directions, uniform distribution of velocities and accelerations, minimizing energy consumption, optimizing execution time, etc. However, there are also such drawbacks, as complicated calculations for IK and motion control, and greater structural complexity of construction. A manipulator is kinematically redundant when it possesses more degrees of freedom than it is needed to execute a given task. In other words, there are more local coordinate variables than global. The Redundancy Resolution is necessary because it allows to avoid singularity, obstacles and to smooth manipulation around the workspace. The solution for the redundancy is a cost function optimization, where the cost function can be i) energy-based  or ii) minimizing the distance. The Redundancy Resolution includes three methods, which are
- Jacobian-based (Damped Least Squarege and Weighted Pseudoinverse)
- Null Space
- Task Augmentation

In this repository, the implementation of forward and inverse kinematics is presented. 


<img src="https://user-images.githubusercontent.com/90580636/171466339-c1a0e96f-71ec-41f2-8891-c0522536ea04.png" width="300" height="200" />

---

### Results

