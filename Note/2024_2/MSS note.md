# MSS

## 一、工程目录

### 1. GNC（Guidance引导，Navigation导航，Control控制）

GNC 的基本库和系统示例。该库包含：

- 运动学和动力学的 M-file 函数，和时域 GNC 应用示例。
  - 比如通过欧拉角计算出四元数，计算旋转矩阵；坐标系的转换等等。
- 包含船舶模型、机动试验和动态仿真的 M 文件库。
  - 比如LQR、锯齿跟踪等（会调用VESSELS中的模型）
- 用户可编辑的 m 文件，用于模拟和控制船舶，包括船舶、半潜式潜水器、自主水下航行器 （AUV）、遥控航行器 （ROV） 和无人水面航行器 （USV）。
  - 比如波浪、风速的影响；一些基本的控制器（lqr，pid）；参考轨迹的计算。
- 用户可编辑的 m 文件，用于惯性导航系统的误差状态卡尔曼滤波。

总而言之，GNC 是一个基础库，一些基本参数的设置（重力加速度，风速波速）；参考系和角度的变换；基础的控制等等。后续的建模和仿真都需要调用 GNC 的文件。

### 2. Hydro（流体力学）

该工具箱读取流体动力学程序生成的输出数据文件，并处理数据以在 Matlab 中使用。MSS Hydrodynamics包括几个示例容器。若要生成模型，需要使用以下程序之一：

- 2D strip theory programs - ShipX (Veres) by SINTEF OCEAN AS
- 3D potential theory programs - WAMIT by WAMIT Inc.

处理后的数据可用于使用位于 /MSS/SIMULINK/mssWamitShipXTemplates/ 下的 Simulink 模板对暴露于一阶和二阶波荷载（运动和力 RAO 传递函数）的 6 个自由度的船舶进行实时仿真。

### 3. FDI

这个工具箱用于识别海上器械（如船舶和波浪能转换器）的辐射力模型和流体记忆效应。与研究内容无关。

### 4. VESSELS（船只模型）

该文件夹内包含了许多船只和 AUV 模型，也包含了一些模型的简单仿真。



综上所述，需要使用的库主要是1和4，用于建模和仿真。



## 二、REMUS100 PID 控制

### 1. REMUS100 建模

![02-01](.\image\02-01.png)

如何对 X-rudder 建模？

可以参考：

> Design and simulation of X-rudder
>
> Handbook of Marine Craft Hydrodynamics and Motion Control. 2nd. Edition, Wiley. URL: www.fossen.biz/wiley

需要的参数：

- AUV 的长度和直径
- 质量分布矩阵、科里奥利矩阵
- 舵面的阻力、横荡力、垂荡力等等



或者使用十字舵的模型，将舵角修改成成X舵的形式（把原来的 $\tau_{w,q}$ 修改成 $\delta_{1-4}$ ）：

![02-03](E:\Library\硕士实验室\MPC\Note\2024_2\image\02-03.png)





### 2. PID

![02-02](.\image\02-02.png)



## 三

1. 找一下地形跟踪相关论文

2. 模拟地形跟踪情景

3. X-rudder 建模或者十字舵

4. 最好的是参考硕士论文

了解地形跟踪方法，进行情景模拟；十字舵、X舵建模，要有参考文献。
总之要把情景搭建起来。