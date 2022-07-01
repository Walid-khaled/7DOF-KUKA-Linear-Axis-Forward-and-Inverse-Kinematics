[[Demo]](https://youtu.be/QU4yxvBdSNA)

## 7DOF-KUKA-Linear-Axis-Forward-and-Inverse-Kinematics
KUKA on linear axis includes a further axis to the robot, so it is considerably extending the workspace of the robot. The advantages of the redundant robots are increasing manipulability in specified directions, uniform distribution of velocities and accelerations, minimizing energy consumption, optimizing execution time, etc. However, there are also such drawbacks, as complicated calculations for IK and motion control, and greater structural complexity of construction. 

A manipulator is kinematically redundant when it possesses more degrees of freedom than it is needed to execute a given task. In other words, there are more local coordinate variables than global. The Redundancy Resolution is necessary because it allows to avoid singularity, obstacles and to smooth manipulation around the workspace. The solution for the redundancy is a cost function optimization, where the cost function can be i) energy-based  or ii) minimizing the distance. The Redundancy Resolution includes three methods, which are
- Jacobian-based (Damped Least Square and Weighted Pseudoinverse)
- Null Space
- Task Augmentation

In this repository, the implementation of forward and inverse kinematics by redundancy resolution is presented.

<p float="left">
  <img src="https://user-images.githubusercontent.com/90580636/171466339-c1a0e96f-71ec-41f2-8891-c0522536ea04.png" width="300" height="200" />
  <img src="https://user-images.githubusercontent.com/90580636/171468054-fc96ca19-5f39-4893-9723-24872b333bb4.png" width="450" height="200" />
</p>

### Forward Kinematics can be written as follows:
![image](https://user-images.githubusercontent.com/90580636/171468322-838cf1ab-ec6d-4d20-b146-d6a899b7f772.png)

## Redundancy Resolution
<p float="left">
  <img src="https://user-images.githubusercontent.com/90580636/171469073-93323e4f-abcc-469c-b489-19b9a1db5b5d.png" width="400" height="250" />
  <img src="https://user-images.githubusercontent.com/90580636/171469345-cb54277e-f549-4ec6-8ec3-556dd4845b13.png" width="400" height="250" />
</p>

<p float="left">
  <img src="https://user-images.githubusercontent.com/90580636/171469849-489290aa-b067-4952-9732-b77974ec8a10.png" width="400" height="250" />
  <img src="https://user-images.githubusercontent.com/90580636/171469928-d7b110be-e9cd-44af-a386-8c02b60a546d.png" width="400" height="250" />
</p>

<p float="left">
  <img src="https://user-images.githubusercontent.com/90580636/171470153-220e32db-2a84-4a0e-a2dc-6f905537f914.png" width="400" height="250" />
  <img src="https://user-images.githubusercontent.com/90580636/176934874-4ce5ce3d-e58e-46a2-8a39-a9e0eca66644.gif" width="400" height="250" />
</p>

Full video is attached [Demo](https://youtu.be/QU4yxvBdSNA)

## Accuracy and Execution Time
<p float="left">
  <img src="https://user-images.githubusercontent.com/90580636/171470183-2ce3e89f-c301-4c04-a6e9-2bafcdb007ac.png" width="400" height="250" />
  <img src="https://user-images.githubusercontent.com/90580636/171470229-4b67a4a2-a5a0-4eb3-bdb6-4ea947d22362.png" width="400" height="250" />
</p>

## How to run
First open **IK.m** file and select the preferable redundancy resolution approach as shown below.  
![image](https://user-images.githubusercontent.com/90580636/176931082-95879408-ef2a-4dbc-ab4c-08d87c0bbe02.png)

Then run **main_redundancy_resolution.m** file for calculations and visualization.

