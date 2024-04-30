## Piecewise

### 表征稳定状态和稳定输入

- 第一步：写出稳态方程

  ![19](E:\Note\MPC\Note\2023_12\image\19.png)

  该方程有多个解，令 $z_s=[x_s^T\ \ \ \ u_s^T]^T$ ，方程组 $[A-I\ \ \ \ B]\ z_s=0$ 的解为 $[A-I\ \ \ \ B]$ 的零空间，对其SVD分解后就是 $R(V_2)$ 。直接令 $M_\theta = V_2$ ，参数 $\theta \in R^{n_{\theta}}$ ，那么 $R(V_2)$ 可以表示为 $M_{\theta}\theta$ ，即 $z_s=M_{\theta}\theta$ 。

  那么 $y_t=[C\ \ \ \ D]\ z_s=[C\ \ \ \ D]\ M_{\theta}\theta$ ，令 $N_\theta=[C\ \ \ \ D]\ M_{\theta}$  ，即可表示为：
  $$
  z_s=M_\theta \theta\\
  y_t=N_\theta \theta
  $$

  

- 第二步：结合系统约束，写出稳态约束

  系统约束：$(x_k,u_k)\in Z=\{z\in R^{n+m}:A_zz\leq b_z\}$

  稳态约束：$Z_s=\{(x_s,u_s)=M_{\theta}\theta:M_{\theta}\theta\in Z\}$

### 计算跟踪不变集

- 第一步：设计控制律
  $$
  u=K(x-x_s)+u_s=Kx+L\theta\\
  L=[-K\ \ \ I_m]M_{\theta}
  $$

  

- 第二步：将 $\theta$ 增广至系统状态
  $$
  \begin{align}
  w&=(x^T,\theta^T)^T\\
  A_w&=\begin{bmatrix}
  A+BK&BL\\
  0&I_{n\theta}
  \end{bmatrix}\\
  w^+&=A_ww
  \end{align}
  $$

  

- 第三步：给出不变集的定义式

  定义凸集：$W_{\lambda}=\{w=(x,\theta):(x,Kx+L\theta)\in Z,M_{\theta}\theta\in\lambda Z\}$

  该集合既保证了系统约束，也保证了稳态约束

  给出最大不变集的定义式：$O_{\infty}^w=\{w:A_w^iw\in W_1,\forall i\geq 0\}$

  

- 第四步：有限确定的不变集

  `Linear systems with state and control constraints` 中的定理 5.1 ：

  ![21](E:\Note\MPC\Note\2023_12\image\21.png)

  ![22](E:\Note\MPC\Note\2023_12\image\22.png)

  $O_{\infty}(A,\hat{C},Y(\epsilon)\times Y)$ 可有限决定。

  在本文中使用了此定理。

  $O_{\infty,\lambda}^w=\{w:A_w^iw\in W_{\lambda},\forall i\geq 0\}$ 可有限决定。 

### 跟踪 MPC

成本函数：

![23](E:\Note\MPC\Note\2023_12\image\23.png)

优化问题：

![24](E:\Note\MPC\Note\2023_12\image\24.png)

其中 $\hat{x}_s$ 是人工指定的目标稳态。与标准 MPC 不同的是，惩罚了人工稳态和计算得到的稳态的偏差，以及控制输入和控制稳定控制输入的偏差；此外还增加了一个终端约束用于惩罚人工稳态和计算得到的稳态的偏差。

这个优化问题不依赖于人工稳态的选取，存在一个凸集 $X_N$ ，初始状态在该区域内的优化问题都有解。那么假设 2 中的 $X_f=Proj_x(X_f^w)\subseteq X_N$ 。



## Reference Management

离散时域模型：

<img src="E:\Note\MPC\Note\2024_4\image\01-01.png" alt="01-01" style="zoom:80%;" />

控制目标：

<img src="E:\Note\MPC\Note\2024_4\image\01-02.png" alt="01-02" style="zoom:80%;" />

考虑系统约束以及扰动，对稳态值的要求如下：

<img src=".\image\01-03.png" alt="01-03" style="zoom:80%;" />

### Admissible Commands（reference）

通过引入任意小的公差向量 $\epsilon_x$ 和 $\epsilon_z$，状态空间方程和目标方程所代表的等式约束可以转换为不等式约束：

<img src="E:\Note\MPC\Note\2024_4\image\01-09.png" alt="01-09" style="zoom:80%;" />

那么就可以用一个集合来表示：

<img src="E:\Note\MPC\Note\2024_4\image\01-10.png" alt="01-10" style="zoom:80%;" />

假定扰动可通过某个观测器获得。因此，通过对估计的干扰值进行**切片**运算，就可以轻松计算出 $r_{ss}$ 的极限值（因为有降维）。切片运算**简化了线性不等式**，当某些变量先验已知时，线性不等式定义了一个多面体。因此，切片操作会导致多面体的维度降低。投影算法在实时应用中限制于低阶多面体。不过，通过假设不同的故障组合情况会产生不同的矢量 $k_{ss}$，而每个矢量的 $u_{max}$ 和 $u_{min}$ 值都不同，离线确定可操控性多面体似乎很有用。对估算出的干扰矢量进行切片是一个非常简单的操作，可以实时执行。

因此，在给定一个执行器故障情况下，可通过公式（10）获得相关的可变多面体 $P_m$。每个故障组合都与一个多面体 $P_m$ 相关联，由于只用一个矩阵和一个矢量表示，因此可以很容易地存储起来（Lss和kss），以便在实时应用中使用。因此，只需进行简单的操作，就能验证给定参考 $r_{ss}$ 的可行性。最后，对所选公差做一些说明。为了保持状态和输出约束的完整性，公差值越小越好。然而，过小的公差可能会导致投影算法不可行，从而产生空多面体。因此，必须谨慎选择这些参数，以确保构建出非空的可操控性多面体。

根据该集合，可以得到 $r_{ss}$ 的容许范围。

### Feasible Target-Tracking Model Predictive Control

控制律：

<img src="E:\Note\MPC\Note\2024_4\image\01-12.png" alt="01-12" style="zoom:80%;" />

MPC 计算的是 $c_j$ ，$K_d$ 是固定的。

假设稳态值（xss,uss）固定，那么将状态向量增广，并将控制律代入系统模型：

<img src="E:\Note\MPC\Note\2024_4\image\01-13.png" alt="01-13" style="zoom:80%;" />

对应的约束：

<img src="E:\Note\MPC\Note\2024_4\image\01-14.png" alt="01-14" style="zoom:80%;" />

根据参考文献[4]可以计算出不变集，根据不变集可以计算出终端约束 $X_N$ 。

<img src="E:\Note\MPC\Note\2024_4\image\01-15.png" alt="01-15" style="zoom:80%;" />

终端约束是为了保证 MPC 有可行解，那么只要让给定的稳态目标 $x_{ss}$ 和 $u_{ss}$ 在终端约束中即可：

<img src=".\image\01-16.png" alt="01-16" style="zoom:80%;" />



## Contrast

Piecewise：通过参数 $\theta$ 表征稳态解，在一个 MPC 优化问题中解决了两件事：

1. $x_{ss}$ 的选择
2. 控制律计算。

$x_{ss}$ 的选择就相当于参考管理的功能。

RG：相比于 Piecewise 来说多了一个 reference（Piecewise 直接使用的 $\hat{x}_s$），那么就多了一层：从 Admissible Commands（ref） 到 $x_{ss}$ 的对应计算。RG 有三个模块：

1. 离线计算 ref 的容许集合，对 ref 进行过滤。（与 Piecewise 的 1 对应）
2. 根据过滤后的 ref 计算对应 recursive feasibility 的稳态解 $x_{ss}$ 。
3. MPC 控制律计算。（与 Piecewise 的 2 对应）

两个算法本质上都做到了 reference management ，区别在于 Piecewise 在一个优化问题里同时做到了参考管理和控制律的计算，这是通过增广状态向量做到的。

Piecewise 中这种把“reference management”和控制的优化放在一个优化问题里解决的实现方式，对于“跟踪参考轨迹”这个目标来说是次优的，因为 4 个权重 P, Q, R, T 的设置，给定的参考指令 $\hat{x}_s$ 的优化权重 $T$ 不可能无限大，所以计算得出的 $x_s$ 只是相对成本函数的最优解。

而 RG 把 reference management 和控制分开，先通过过滤得到最优的 ref ，再进行之后的控制。Piecewise 这种合起来计算优化的方式能够减少计算时间，然而 RG 虽然分开计算，但是它的其中一步是不需要优化的，直接用线性投影、切片就可以得出结果，所以几乎没有牺牲计算速度。