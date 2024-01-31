### 大纲

1. AUV 的应用领域
2. 纵向面建模，控制分配
3. 针对轨迹跟踪设计
4. 针对执行器故障的容错控制设计，如何进行故障检测



### 应用领域

1. 水下干预

   [关于Sauvim：概述 (gmarani.org)](https://gmarani.org/na/sauvim/Overview.html)

   <img src=".\image\03-01.png" alt="03-01" style="zoom:25%;" />

2. 目标跟踪

3. 海洋调查（海底测绘等）

### 纵向面建模

`Optimal robust trajectory tracking control of a X-rudder AUV with velocity  sensor failures and uncertainties`

对于海底水下测绘或者零纵倾控制，需要对 X-AUV 的纵向面进行建模。

运动学方程：

![03-02](.\image\03-02.png)

$x,z,\theta$ 代表水平位置，竖直位置和方向。$u,w,q$ 代表推进速度，倾斜速度和俯仰角加速度。

纵向面动力学模型：

![03-03](.\image\03-03.png)

$m,W,B$ 代表设备的质量，重力和浮力。$(x_g,z_g),(x_b,z_b)$ 分别代表设备的重心和浮心。$I_{yy}$ 是设备绕 y 轴的惯性矩。$X_*,Z_* , M_*$ 是设备的流体力学系数。$X_T$ 是沿航行器 **surge 方向**的螺旋桨推力，$τ_w$ 和 $τ_q$ 分别是沿航行器倾斜运动和俯仰运动的舵力和扭矩。$D_u,D_w , D_q$ 表示上界未知的复合环境干扰。

将以上动力学方程重写为：

![03-04](.\image\03-04.png)

$τ_w$ 和 $τ_q$ 到四个执行器的映射为：

![03-05](.\image\03-05.png)



<img src=".\image\03-06.png" alt="03-06" style="zoom:50%;" />

如图 2 所示，AUV 需要在垂直面上跟踪一条所需的轨迹，其中 {I}、{B} 和 {F} 分别表示惯性参考系、AUV 固定参考系和 Frenet-Serret 参考系。P 是要跟踪轨迹上的一个移动点。在所需轨迹上与 P 相关联的是 {F}，其坐标轴与轨迹相切且为法线。

对于轨迹跟踪任务，惯性参考系中的跟踪误差可以表示为：

<img src=".\image\03-07.png" alt="03-07" style="zoom:80%;" />

随后，在 {F} 中建立的轨迹跟踪误差矢量为

![03-08](.\image\03-08.png)

根据此公式和系统运动学方程，得到：
$$
\begin{align}
\dot{x}_e&=\dot{x}_e^I\cos{v_d}-x_e^I\sin{v_d}\dot{v}_d-\dot{z}_e^I\sin{v_d}-z_e^I\cos{v_d}\dot{v}_d\\
		 &=(\dot{x}-\dot{x}_d)\cos{v_d}-\dot{v}_d(x-x_d)\sin{v_d}-(\dot{z}-\dot{z}_d)\sin{v_d}-\dot{v}_d(z-z_d)\cos{v_d}\\
		 &=(u\cos{\theta}+w\sin{\theta}-\dot{x}_d)\cos{v_d}-\dot{v}_d(x-x_d)\sin{v_d}
		   -(-u\sin{\theta}+w\cos{\theta}-\dot{z}_d)\sin{v}_d-\dot{v}_d(z-z_d)\cos{v_d}\\
		 &= u\cos{(\theta-v_d)}+w\sin{(\theta-v_d)}-U_d(1+kz_e)
\end{align}
$$
同理：

<img src=".\image\03-09.png" alt="03-09" style="zoom:80%;" />

因此，轨迹跟踪的动力学方程为：

![03-10](.\image\03-10.png)

根据运动学方程和动力学方程开发一种控制器，以调节自动潜航器在垂直面上跟踪所需的轨迹。
在垂直面上跟踪所需的轨迹，而不受速度传感器故障、未知外部干扰和复杂执行器动态的影响。该目标可分为两个控制问题。
(1) 运动学控制： 给定一个期望轨迹和轨迹跟踪误差模型 (7-8)，推导出一个有效的运动学控制法则，以产生所需的推进速度 $u_d$ 和俯仰角速度 $q_d$，从而使轨迹跟踪误差矢量在  ∞ 时收敛为零。
(2) 动力学控制： 给定所需的浪涌速度 $u_d$ 和俯仰角速度 $q_d$ 以及动力学模型 (3-4)，推导出有效的动力学控制法则，以产生控制输入，从而使跟踪误差 $u - u_d$ 和 $q - q_d$ 在 ∞ 时趋于零。

### 滑膜控制

[【控制理论】滑模控制解析](https://blog.csdn.net/xiaohejiaoyiya/article/details/90271529)

### MPC AUV

`Trajectory Tracking Control of an Autonomous Underwater Vehicle Using Lyapunov-Based Model Predictive Control`

这篇文章所面向的AUV用于水平面的轨迹跟踪，与 `纵向面建模` 类似，首先建立水平面运动学方程：

<img src=".\image\03-11.png" alt="03-11" style="zoom:67%;" />

这是一个类似绕着 z 轴旋转的旋转方程，作用是表达了 AUV 在水平面的速度和朝向。

动力学方程：

<img src=".\image\03-12.png" alt="03-12" style="zoom:67%;" />

这里的 $\tau=Bu$ 代表“控制分配”。

根据这两个方程，可以建立轨迹跟踪动态方程：

<img src=".\image\03-13.png" alt="03-13" style="zoom:67%;" />

根据该状态空间方程，控制 AUV 水平面的运动。



### 容错控制

`X舵AUV控制分配优化与容错控制方法研究`

#### 故障诊断和定位

使用 SVM 进行训练，得到正常情况和故障情况下的分类，用于故障诊断。诊断出故障后，通过让所有舵面归零的方式定位卡死舵。 

<img src=".\image\03-16.png" alt="03-16" style="zoom:67%;" />

#### 容错控制

通过重新进行舵角分配来补偿卡舵。

正常情况下：

<img src=".\image\03-14.png" alt="03-14" style="zoom:50%;" />

卡舵情况下：

<img src=".\image\03-15.png" alt="03-15" style="zoom:50%;" />


