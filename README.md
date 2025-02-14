<img src="./assets/logo.png" style="zoom:30%;" />

# OpenLoong Dynamics Control

## Motion control framework for humanoid robots based on MPC and WBC

Please visit 🐉 OpenLoong open source code repository!

The OpenLoong open-source project is a collaborative initiative operated by Humanoid Robotics (Shanghai) Co., Ltd., the
Shanghai Humanoid Robotics Manufacturing Innovation Center, and the OpenAtom Foundation. This repository provides a
humanoid robot control framework based on MPC (Model Predictive Control) and WBC (Whole-Body Control), which can be
deployed on the Mujoco simulation platform. Based on the "Qinglong" robot model from the Shanghai Humanoid Robotics
Innovation Center, it offers three motion examples: [walking](https://atomgit.com/openloong/openloong-dyn-control/blob/master/demo/walk_wbc.cpp),
[jumping](https://atomgit.com/openloong/openloong-dyn-control/blob/master/demo/jump_mpc.cpp), and [blind obstacle stepping](https://atomgit.com/openloong/openloong-dyn-control/blob/master/demo/walk_mpc_wbc.cpp). The physical
prototype has achieved <b>walking</b> and <b>blind obstacle stepping <b>motions.

📖 **[Read this in Chinese / 阅读中文版](./README-zh.md)**

## Project Features

- **​Easy Deployment** Provides a comprehensive solution for deploying code execution  environments, enabling users to easily configure their required working  environments. This code repository includes major dependencies,  eliminating the need for extensive third-party library installations and  simplifying the entire deployment process.
- **Extensible** The control framework structure adopts a layered modular design aimed at enhancing system maintainability and extensibility. Each functional module of the system has clear logical and functional boundaries, offering a more developer-friendly environment for secondary development. This allows developers to more easily customize and extend system functionalities.
- **Easy to Understand** The code structure is concise, adhering to the principle of modular  encapsulation based on functionality. It uses a bus for data interaction  between modules, reducing encapsulation redundancy and helping to lower  code complexity. Algorithm implementations follow a simple  "read-compute-write" logic, enhancing code comprehensibility.
  
  <center><img src="./assets/行走.gif" alt="行走" style="zoom:50%;" /><img src="./assets/踩障碍.gif" alt="踩障碍" style="zoom:50%;" /></center>

## Changelog

2024.06.29

1. Added two new demos, `walk_wbc_joystick` and `walk_mpc_wbc_joystick`, which allow keyboard-controlled robot movement and enable turning functionality.

2024.08.12

1. Fixed the issue of incorrect sensor data extraction IDs in MuJoCo. Thanks to Xunlong Software for reporting this issue.
2. Corrected the dimensionality error in the definition of the c matrix in  MPC. Thanks to @geekloong and @yichuanku for reporting this issue.
3. Fixed the calculation error in the first priority level of the WBC  priority computation. Thanks to @1190201119 for reporting this issue.
4. Modified the cost function in MPC.

2024.09.11

1. Added a new branch named "low\_damping\_model," which aligns closely with  the joint response of the physical prototype. This branch provides two  demos: `walk_wbc_joystick` and `walk_mpc_wbc_joystick`.
2. Added the **Model Replacement** documentation[Tutorial](https://atomgit.com/openloong/openloong-dyn-control/blob/master/Tutorial.md)。

## Environment Installation

**Environment Suggestion**

- Operation System：Ubuntu 22.04.4 LTS
- Compiler：g++ 11.4.0

**Dependent Installation**

This repository is based on MuJoCo for simulation testing of the "Qinglong" humanoid robot. The repository also includes the MuJoCo simulation engine, the Pinocchio dynamics library, Eigen, the Quill logging tool, the GLFW graphics library, and the JsonCpp parsing library. However, the simulation interface requires system support for OpenGL, which needs to be installed.

```Bash
# Update & Install Dependencies
sudo apt-get update
sudo apt install git cmake gcc-11 g++-11
sudo apt install libglu1-mesa-dev freeglut3-dev
```

## Installation Guide 

**Code Acquisition and Compilation**

```Bash
# Clone
git clone https://atomgit.com/openloong/openloong-dyn-control.git

# Build
cd openloong-dyn-control
mkdir build
cd build
cmake ..
make

# mujoco simulation
./walk_mpc_wbc #or ./walk_wbc or ./jump_mpc
```

**Simulation performance**

<img src="./assets/demo.png" alt="demo" style="zoom:50%;" />

## **Code Explanation**

Refer to the API interface of this code.[Document](https://www.openloong.org.cn/pages/api/html/index.html)and[Wiki](https://www.openloong.org.cn/pages/wiki/html/index.html)。

**Explanation of Main Prefixes and Suffixes**

| Prefixes and Suffixes         | Explanation                      |
| ---------------- | -------------------------- |
| *_L, _W*         | in local frame、in global frame |
| *fe_*            | endpoint of feet                     |
| *_L, _l, _R, _r* | left|right               |
| *swing,* *sw*    | swing leg               |
| *stance,* *st*   | support leg                     |
| *eul, rpy*       | Attitude Angle                     |
| *omega*          | angular velocity                   |
| *pos*            | position                      |
| *vel*            | linear velocity                  |
| *tor**, tau*     | torque                      |
| *base*           | *BaseLink*                 |
| *_des*           | Expected Value                     |
| *_cur*           | cur                 |
| *_rot*           | 坐标变换矩阵               |

## Development Guide

**Key Control Parameters**

- MPC Weights

```C++
//MPC.h
void    set_weight(double u_weight, Eigen::MatrixXd L_diag, Eigen::MatrixXd K_diag);
//*u_weight* ：Minimum weight of system input
//*L_diag* ：Weight of system state and desired error, ordered as eul, pos, omega, vel
//*K_diag* ：Weight of system input, ordered as fl, tl, fr, tr
```

- WBC Task Priorities

```C++
//WBC_QP.cpp
std::vector<std::string taskOrder;
taskOrder.emplace_back("RedundantJoints");
taskOrder.emplace_back("static_Contact");
taskOrder.emplace_back("Roll_Pitch_Yaw_Pz");
taskOrder.emplace_back("PxPy");
taskOrder.emplace_back("SwingLeg");
taskOrder.emplace_back("HandTrack");
//Adjust task priority order
```

- WBC Weights

```C++
//PriorityTasks.h
Eigen::MatrixXd Kp;                // Position error weight for a specific WBC priority
Eigen::MatrixXd Kd;                // Velocity error weight for a specific WBC priority
//WBC_QP.h
Eigen::MatrixXd Q1;                // External contact force and expected error weight, ordered as fl, tl, fr, tr
Eigen::MatrixXd Q2;                // Joint acceleration and expected error weight
```

- Swing Leg Trajectory

```C++
//FootPlacement.h
double kp_vx;                                 // Adjustment parameter for foot placement in x direction
double kp_vy;                                 // Adjustment parameter for foot placement in y direction
double kp_wz;                                 // Adjustment parameter for foot placement in z direction
double stepHeight;                            // Step height
//FootPlacement.cpp
double    FootPlacement::Trajectory(double phase, double des1, double des2);        //Swing Leg Trajectory in z direction
//phase：Swing phase reaching the highest point
//des1：Highest trajectory position
//des2：Final trajectory position
```

- Gait Control

```C++
//GaitScheduler.h
double tSwing;                                         // Single step duration  
double FzThrehold;                                     // Ground contact force threshold
//GaitScheduler.cpp
DataBus::LegState legState=DataBus::RS;                // Initial swing leg
```

- Joint Parameters

```json
//JointCtrConfig.json
   "Joint-ankle-l-pitch" : {
      "PVT_LPF_Fc" : 20,
      "kd" : 5.0,
      "kp" : 50.0,
      "maxPos" : 0.61087,
      "maxSpeed" : 48.8,
      "maxTorque" : 58.5,
      "minPos" : -0.43644
   }
```

**Model Replacement Guide**

Refer to the[Tutorial](https://atomgit.com/openloong/openloong-dyn-control/blob/master/Tutorial.md)document for model replacement.

## Reference

[1] D. Kim, J. D. Carlo, B. Katz, G. Bledt, S. Kim, Highly dynamic quadruped locomotion via whole-body impulse control and model predictive control. arXiv:1909.06586 (2019).

[2] Kim D, Jorgensen S J, Lee J, et al. Dynamic locomotion for passive-ankle biped robots and humanoids using whole-body locomotion control. arXiv:1901.08100 (2020).

[3] Di Carlo J, Wensing P M, Katz B, et al. Dynamic locomotion in the mit  cheetah 3 through convex model-predictive control[C]//2018 IEEE/RSJ  international conference on intelligent robots and systems (IROS). IEEE, 2018: 1-9.

[4] 卞泽坤, 王兴兴. 四足机器人控制算法: 建模、控制与实践[M]. 机械工业出版社, 2023

## Citation Format

If you use the code from this open-source project, please cite it as follows:

```JavaScript
@software{Robot2024OpenLoong,
  author = {Humanoid Robot (Shanghai) Co., Ltd},
  title = {{OpenLoong-DynamicsControl: Motion control framework of humanoid robot based on MPC and WBC}},
  url = {https://atomgit.com/openloong/openloong-dyn-control.git},
  year = {2024}
}
```

## Contact Information

Developers are welcome to contribute to the optimization and improvement of this code repository!

[💬 Start a Discussion](https://atomgit.com/openloong/openloong-dyn-control/discussions/new/choose) | [📝 Report an Issue](https://atomgit.com/openloong/openloong-dyn-control/issues/create) | [📨 [Submit a Change Request](https://atomgit.com/openloong/openloong-dyn-control/changes)](https://atomgit.com/openloong/openloong-dyn-control/changes)

For any questions or suggestions regarding this code, please contact<web@openloong.org.cn>

