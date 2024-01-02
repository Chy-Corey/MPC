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

  

- 第五步

