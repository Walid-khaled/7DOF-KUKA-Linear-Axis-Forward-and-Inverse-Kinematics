## KUKA-Linear-Axis-Forward-and-Inverse-Kinematics
This repository contains the implementation of the stiffness analysis of the double pantograph transmission system. It is implemented using VJM and MSA techniques to fnd the EE deﬂection at different points in the workspace. After deﬂection calculations, deﬂection scatter plots are built to analyse the maximum deﬂection due to 100N force along x, y, z directions respectively. It was found that VJM method has some limitations to model this structure, which prevent evaluating the deﬂection with an external load in x, z directions. A comparative analysis is performed to compare both MSA and VJM computation complexity and deﬂection deference. It was noticeable that MSA has less computation cost and no limitations to model the double pantograph structure compared to VJM approach.

---

### Results

<img src="https://user-images.githubusercontent.com/90580636/171466339-c1a0e96f-71ec-41f2-8891-c0522536ea04.png" width="150" height="200" />
