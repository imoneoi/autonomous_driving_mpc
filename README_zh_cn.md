# 自动驾驶车辆模型预测控制算法

本项目基于ROS框架开发，包含实时车辆模拟器，模型预测控制器两部分组成。在模拟环境实现稳定地控制自动驾驶车辆跟随道路的功能。

# 编译方法

# Step 1
1. 推荐使用Ubuntu 18.04LTS系统，本项目在该系统下编译及测试。
2. 安装ROS Melodic版本，**此步骤较为繁琐**，请参考ROS Wiki完成安装，具体操作方式详见[这里](http://wiki.ros.org/melodic/Installation)，需要desktop-full安装，在此不在赘述。
3. 安装ecl_geometry

```bash
sudo apt install ros-melodic-ecl-geometry
```

# Step 2
下载并构建OSQP

请在本项目根目录输入如下指令

```bash
git clone --recursive https://github.com/oxfordcontrol/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
```

如果有问题请参考[OSQP官方文档](https://osqp.org/docs/get_started/sources.html)

# Step 3
编译本项目

请将本项目放置于ROS工作区中，项目文件夹名字为car_model，关于如何建立ROS工作区请参考[这里](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)，并且在ROS工作区目录输入如下指令

```bash
catkin_make
```

至此，大功告成！

# 使用及测试

打开终端输入

```bash
roslaunch car_model sim.launch
```

即可看到模拟器界面和曲线图

可以看到车辆很稳定地跟随车道

模拟器界面中，红色为预测轨迹，蓝色为规划目标轨迹，绿色为车道中心。

曲线图显示Cross-Track-Error，即车辆偏离车道中心的距离，用于衡量控制器性能。

# 测试方法

本项目子算法部分已经进行单元测试，在各类的runTest方法下面。模型预测控制器的模型部分经过人工检查，OSQP求解器是现成的包，已经单元测试。

测试控制器性能需要通过观察预测轨迹和Cross-Track-Error，预测轨迹越接近实际轨迹，则物理模型性能越好。Cross-Track-Error，即车辆偏离车道中心的距离，是量化指标，越低表示控制器性能越好。

# 结构

```
include ... 头文件
    controller_mpc -- MPC控制器头文件
        mpc.h -- MPC控制器
        path_planner.h -- 轨迹规划程序
        quintic_solver.h -- 五次方程数值求解，使用Descartes Rule of Signs及二分法

    simulator -- 模拟器
        simulator.h -- 模拟器
        vec2.h -- 二维向量库

src
    controller_mpc -- MPC控制器
        controller_mpc.cpp -- MPC控制器 ROS 接口部分
        mpc.cpp -- MPC控制器 主体实现
        path_planner.cpp -- 轨迹规划程序

    simulator -- 模拟器
        main.cpp -- 主程序
        simulator.cpp -- 模拟器算法及车辆仿真模型

tracks
    车道轨迹，每行两个数x,y，表示路径点坐标，程序会自动插值成三次样条。

launch
    roslaunch 启动文件

msg
    ROS 消息定义

osqp
    OSQP 二次规划求解器
```

# 算法

# 参考文献

*链接均有效，如无法打开，请检查网络连接*

**物理模型部分**
1. [Robust Model Predictive Control for Autonomous Vehicle/Self-Driving Cars](https://arxiv.org/pdf/1805.08551.pdf)
2. [Car Model](https://github.com/MPC-Berkeley/barc/wiki/Car-Model)
3. [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)

**控制算法**
1. [Lane keeping in autonomous driving with Model Predictive Control & PID](https://medium.com/@jonathan_hui/lane-keeping-in-autonomous-driving-with-model-predictive-control-50f06e989bc9)

**QP solver**
1. [OSQP solver documentation](https://osqp.org/docs/index.html)

**ROS 框架**
1. [ROS Wiki](http://wiki.ros.org/)