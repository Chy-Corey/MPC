## 2008. MPC for tracking piecewise constant references for constrained linear systems



### Chapter 2.1 表征稳定状态和稳定输入

- 第一步：写出稳态方程

  ![19](.\image\19.png)

  该方程有多个解，令 $z_s=[x_s^T\ \ \ \ u_s^T]^T$ ，方程组 $[A-I\ \ \ \ B]\ z_s=0$ 的解为 $[A-I\ \ \ \ B]$ 的零空间，对其SVD分解后就是 $R(V_2)$ 。直接令 $M_\theta = V_2$ ，参数 $\theta \in R^{n_{\theta}}$ ，那么 $R(V_2)$ 可以表示为 $M_{\theta}\theta$ ，即 $z_s=M_{\theta}\theta$ 。

  那么 $y_t=[C\ \ \ \ D]\ z_s=[C\ \ \ \ D]\ M_{\theta}\theta$ ，令 $N_\theta=[C\ \ \ \ D]\ M_{\theta}$  ，即可表示为：
  $$
  z_s=M_\theta \theta\\
  y_t=N_\theta \theta
  $$

  

- 第二步：结合系统约束，写出稳态约束

  系统约束：$(x_k,u_k)\in Z=\{z\in R^{n+m}:A_zz\leq b_z\}$

  稳态约束：$Z_s=\{(x_s,u_s)=M_{\theta}\theta:M_{\theta}\theta\in Z\}$



### Chapter 2.2 计算跟踪不变集

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

  ![21](.\image\21.png)

  ![22](.\image\22.png)

  $O_{\infty}(A,\hat{C},Y(\epsilon)\times Y)$ 可有限决定。

  在本文中使用了此定理。

  $O_{\infty,\lambda}^w=\{w:A_w^iw\in W_{\lambda},\forall i\geq 0\}$ 可有限决定。 



### Chapter 3 跟踪 MPC

在本节中，介绍了建议用于跟踪的 MPC。该预测控制器基于**将稳态和输入添加为决策变量**，使用修改后的成本函数和扩展终端约束。

**假设1：**

- (A,B) 可镇定

**假设 2：**

- Q, R, T 正定
- (A+BK) 赫尔维兹矩阵
- P 正定， $(A+BK)^T(A+BK)-P=-(Q+K^TRK)$
- $X_f^w$ 是增益 K 下的满足系统和约束的容许跟踪不变集

成本函数：

![23](.\image\23.png)

优化问题：

![24](.\image\24.png)

其中 $\hat{x}_s$ 是人工指定的目标稳态。与标准 MPC 不同的是，惩罚了人工稳态和计算得到的稳态的偏差，以及控制输入和控制稳定控制输入的偏差；此外还增加了一个终端约束用于惩罚人工稳态和计算得到的稳态的偏差。

这个优化问题不依赖于人工稳态的选取，存在一个凸集 $X_N$ ，初始状态在该区域内的优化问题都有解。那么假设 2 中的 $X_f=Proj_x(X_f^w)\subseteq X_N$ 。



**定理1：**

当假设 1 和 2 成立时，取 $X_f^w=O_{\infty,\lambda}^w$ 。考虑 $\hat{x}_s$ 是可行的，那么对于任何可行的初始状态 $x_0\in x_N$ ，该控制器可将系统状态稳定到 $\hat{x}_s$ 。



**备注1：**

稳态集合 $X_s=Proj_x(Z_s)$ ，$Z_s=\{(x_s,u_s)=M_{\theta}\theta: M_{\theta}\theta\in Z \}$

增广稳态集合 $W_{\lambda}=\{w=(x,\theta):(x,Kx+L\theta)\in Z,M_{\theta}\theta\in\lambda Z\}$，

最大不变集 $O_{\infty,\lambda}^w=\{w:A_w^iw\in W_{\lambda},\forall i\geq 0\}$ ，其中的 x 的集合为 $O_{\infty,\lambda}=Proj_x(O_{\infty,\lambda}^w)$ 。

由于 $X_S\subset O_{\infty,1}$ （稳态集合比不变集小），所以 $\lambda X_S\subset \lambda O_{\infty,1}\subset O_{\infty,\lambda}$ 。

所以 $\lambda X_S\subset X_f\subset X_N$ ，也就是说给定的稳态 $\hat{x}_s$ 如果在 $\lambda X_s$ 中，就在不变集中，可以进行无偏跟踪。



**备注2：**

考虑给定的可接受目标稳定状态 $\hat{x}_s\in \lambda X_s$ ，并设计一个标准MPC，将最大容许不变集 $O_{\infty}(\hat{x}_s)$ 设置为终端约束。考虑本文提出的 MPC，使用相同的参数（Q、R、P、K 和 N）和终端约束 $O_{\infty,\lambda}^w$ 。

1. 以 $\hat{x}_s$ 作为目标点的不变集 $O_{\infty}(\hat{x}_s)$ 表示能够收敛到 $\hat{x}_s$ ，$O_{\infty}(\hat{x}_s) \subset O(\infty,\lambda)$ 。两者作为终端约束时，谁更大吸引域则越大。说明本文提出的方法拥有更大的吸引域。
2. 由于成本函数要求 $x_S$ 与 $\hat{x}_S$ 尽量接近，如果 T 较小，会导致计算的 $x_s$ 距离目标稳态略远，会损失一定的最优性能（不能最快逼近目标稳态），这种最优性损失可以通过增大矩阵T惩罚跟踪误差项解决。



### 定理证明

**引理1：**

![25](.\image\25.png)

$\hat{x}_S$ 是可行的指定稳态，K 是控制增益，P是对应的李雅普诺夫矩阵，保证系统稳定性。

对于稳态 $\bar{x}_s=\lambda x_s+(1-\lambda)\hat{x}_S$ ，满足以上两个性质：

**证明：**

定义椭球体：$E(x_0,\tau)=\{x\in R^n:||x-x_0||^2_P\leq \tau \}$

令 $z_s=(x_s,u_s)=M_{\theta}\theta$ ，$\hat{z}_s=(\hat{x}_s,\hat{u}_s)=M_{\theta}\hat{\theta}$ ，$\bar{z}_s=(\bar{x}_s,\bar{u}_s)=M_{\theta}\bar{\theta}$ 。对应的，$\bar{\theta}=\lambda \theta+(1-\lambda)\hat{\theta}$ 。

注意 $(x_s,u_s)$ 在 $Z$ (Z是系统约束)的内点中，那么一定存在一个合适的常数 $\epsilon>0$ 和 $\gamma\in (0,1)$ 满足：
$$
\forall x\in E(x_s,\epsilon),(x,Kx+L\theta)\in \gamma Z
$$
可以理解为内点及其周围可以组成一个球体，被包含在 Z 里，并且可以用 $\epsilon$ 和 $\gamma$ 来量化其大小。

给定一个充分大的 $\lambda \in (0,1)$ ，使：
$$
\begin{align}
(0,L(\bar{\theta}-\theta))&=(0,L(\lambda \theta+(1-\lambda)\hat{\theta}-\theta))\\
&=(0,L((1-\lambda)(\hat{\theta}-\theta)))\\
&\in (1-\gamma)Z
\end{align}
$$
$\lambda$ 需要充分大，是要让 $\bar{\theta}$ 距离 $\theta$ 充分近，保证在区域 $ (1-\gamma)Z$ 内 。

找一个 $\beta>0$ ，满足 $x_s\in E(\bar{x}_s,\beta)\subset E(x_s,\epsilon)$ 。

对于 $\forall x\in E(\bar{x}_s,\beta)$ ，有：
$$
\begin{align}
(x,Kx+L\bar{\theta})&=(x,Kx+L(\bar{\theta}+\theta-\theta))\\
&=(x,Kx+L\theta)+(x,L(\bar{\theta}-\theta))
\end{align}
$$
由于对 $\forall x\in E(\bar{x}_s,\beta)$ ，有：
$$
(x,Kx+L\theta)\in \gamma Z\\
(0,L(\bar{\theta}-\theta))\in (1-\gamma)Z
$$
所以 $(x,Kx+L\bar{\theta})\in Z$ 成立。 

以 $\bar{x}$ ($\bar{\theta}$) 得到的控制输入 $Kx+L\bar{\theta}$ 满足系统约束，并且 K 和 P 是李雅普诺夫稳定的，所以 x 会一直保留在 $E(\bar{x}_s,\beta)$  中，即 $E(\bar{x}_s,\beta)$ 是 $O_{\infty}(\bar{x}_s)$ 的一个子集：$x_s\in E(\bar{x}_s,\beta)\subset O_{\infty}(\bar{x}_s)$ 。

<img src=".\image\28.png" alt="28" style="zoom:30%;" />



接下来证明第二点，考虑 $\bar{x}_s$ 的定义式： $\bar{x}_s=\lambda x_s+(1-\lambda)\hat{x}_S$ ，可以得到：
$$
x_s-\bar{x}_s=(1-\lambda)(x_s-\hat{x}_s)\\
\bar{x}_s-\hat{x}_s=\lambda(x_s-\hat{x}_s)
$$
将等式两边求范数：
$$
||x_s-\bar{x}_s||_P=(1-\lambda)||x_s-\hat{x}_s||_P\\
||\bar{x}_s-\hat{x}_s||_T=\lambda||x_s-\hat{x}_s||_T
$$
平方求和得到：
$$
\begin{align}
||x_s-\bar{x}_s||_P^2+||\bar{x}_s-\hat{x}_s||_T^2&=(1-\lambda)^2||x_s-\hat{x}_s||_P^2+\lambda^2||x_s-\hat{x}_s||_T^2\\
&=||x_s-\hat{x}_s||_H^2
\end{align}
$$
其中 $H=\lambda^2T+(1-\lambda)^2P$ 。

证明目标就转为了 $||x_s-\hat{x}_s||_H^2<||\bar{x}_s-\hat{x}_s||_T^2$ ，即证明 $T-H>0$ 。

对于 T, P，存在常数 $\gamma>0$ ，使 $P<\gamma T$ 。那么：
$$
\begin{align}
T-H &=(1-\lambda^2)T-(1-\lambda)^2P\\
    &>((1-\lambda^2)-(1-\lambda)^2\gamma)T\\
    &=(1-\lambda)[(1+\lambda)-(1-\lambda)\gamma]T
\end{align}
$$
要使上式大于 0 ，则 $1+\lambda-(1-\lambda)\gamma>0$ ，可以推出 $\lambda>\frac{\gamma-1}{\gamma+1}$ 。第二点证毕。



**引理2：**

![26](.\image\26.png)

对于在系统终端约束中的状态 x ，如果 x 在目标稳态 $x_s$ 的不变集 $O_{\infty}(x_s)$ 内，则满足以上不等式。

证明：

![27](.\image\27.png)

这里的 x 表示 $x(0)$ 。

<img src=".\image\29.png" alt="29" style="zoom:67%;" />

![30](.\image\30.png)



**引理3：**

![31](.\image\31.png)

$\lambda X_s$ 是稳态集合 $X_s=Proj_x(Z_s)$ ，$Z_s=\{(x_s,u_s)=M_{\theta}\theta: M_{\theta}\theta\in Z \}$ ，考虑指定的目标稳态 $\hat{x}_S \in \lambda X_s$ ，如果优化问题 $P_N(x,\hat{x}_S)$ 的最优解使得 $||x-x_s^*||_Q=0$ ，那么 $||x-\hat{x}_s||_T=0$ 。

此引理可以理解为如果人工稳态可达，当系统状态达到目标稳态时，目标稳态等于人工稳态。

**证明：**















