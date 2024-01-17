### Fault-tolerant tracking control optimization of constrained LPV systems based on embedded preview regulation and reference governance



#### abstract

本文通过嵌入**优化预计调节（optimal preview regulation）**和**参考治理（reference governance）**，对相关约束预测容错跟踪控制（fault-tolerant tracking control, FTTC）方法提出了一些新的改进。**这种策略的主要新颖之处在于可以充分调度有限未来参考的一些有价值信息，从而优化鲁棒跟踪性能并显著扩大容错容许区域。**

为了更好地说明所提策略的广泛适用性，我们考虑了一类具有状态/输入约束的 LPV 系统的鲁棒 FTTC 问题。总的来说，所涉及的关键设计包括三个部分。

1. 首先，通过结合跟踪误差反馈、参考输入调节和故障信号补偿，构建了无约束 FTTC 组件。它用于保证闭环系统在约束条件未被激活时的鲁棒跟踪稳定性。
2. 其次，设计了带有嵌入式最优预览调节器的管式预测 FTTC 策略，以实现稳健的约束满足和瞬态响应改善。
3. 第三，额外集成了一个嵌入式参考调节器，以显著扩大容错可容许区域的大小。这一设计进一步加强了约束 FTTC 优化的可行性。这些结果的有效性最终通过单晶体管直流/直流正向转换器的案例研究得到了验证。



#### 问题描述

线性变参数系统：

![2-1](.\image\02-01.png)

该系统的输入增加了 $f_k$ ，代表了未知的执行器偏移或漂移故障向量。

系统矩阵 $A(\theta_k),B(\theta_k),D(\theta_k)$ 是根据调度参数 $\theta_k$ 变化的时变矩阵。可以把系统矩阵的范围限定到一个有 h 个顶点的超立方体中，超立方体为：
$$
\Gamma(\theta_k)=\Sigma_{i=1}^h\theta_{i,k}\Gamma_i,\Gamma_i\in\{ A_i,B_i,D_i \},\Sigma_{i=1}^h\theta_{i,l}=1
$$
备注 1：一般而言，就 FTC 方法的设计复杂性及其应用效果，采用主动还是被动 FTC 方法取决于故障对给定系统性能的影响程度。以执行器故障为例，如果某些执行器偏移故障的影响很小，我们可以将其视为干扰，并通过设计被动 FTC 方法对其进行稳健抑制。相反，如果离线仿真分析表明执行器故障的影响较大或无法忽略，则应采用主动 FTC 方法进行有针对性的消除和容差。本文默认考虑后一种情况。

考虑目标参考指令 $y_{k+1}=r_{k+1}$ ，对应的状态和输入的稳定点为：
$$
x_{r,k}=K_{xr}(\theta_k)r_{k+1}\\
u_{r,k}=K_{u}(\theta_k)r_{k+1}
$$
假设 2：参考指令 $r_k$ 在有限的预计时域（preview horizon）$n_r$ 内是可预计的，在时间 $k$ 可以知晓 $r_k,\dots,r_{k+n_r}$。

假设3：扰动向量 $w_k$ 在一个紧凑的凸集中：$W=\{ w||w_k|\leq \varrho_\omega\}$ 。故障向量的变化率 $\delta_{f,k}=f_{k+1}-f_k$ 也满足：$F=\{ \delta_f||\delta_{f,k}|\leq \varrho_f\}$. 



目标是设计一个带约束的最优预测容错跟踪控制策略，能够使系统输出跟踪 $r_k$ ：
$$
\begin{align}
&J_k=\Sigma_{t=0}^{\infty}U(x_{k+t},\ \ \ \ u_{k+t}),s.t.\ \ x_{k+t}\in X,u_{k+t}+f_{k+t}\in U \\
& U(x_{k+t},u_{k+t})=(x_{k+t}-x_{r,k+t})^TQ(x_{k+t}-x_{r,k+t})+(u_{k+t}+f_{k+t}-u_{r,k+t})^TR(u_{k+t}+f_{k+t}-u_{r,k+t})
\end{align}
$$


如果不考虑约束，无限时域 LQR 求解得到的控制率形式为：
$$
u_k=K_e(x_k-x_{r,k})+u_{r,k}-\hat{f}_k
$$
本文并没有用 MPC 的方法来计算控制输入，而是预先给定无约束下的最优控制律，并通过 $c_k$ 来保证可解性和满足约束：

![02-02](.\image\02-02.png)

$c_k$ 是组合了最优的预计调节器（preview regulator, PR）和参考治理（reference governance, RG）的预测输入。

 

将控制律表示为：

![02-03](.\image\02-03.png)

$K_r\vec{r}_{k+1}$ 表示最优的预计调节器（preview regulator, PR），通过考虑后续参考指令来增强瞬态响应性能（前馈）。

$\zeta_k$ 使嵌入式参考治理（reference governance, RG）成为可能，并用于加强约束优化的稳态可行性。

预测扰动 $K_c\eta_k=[I\ \ \ \ 0]\vec{\eta}_k=\eta_k$ 将 $\vec{\eta}_k$ 的第一个最优值移动置于 $u_k$ 中，用于保证瞬态响应的约束满足。

前馈 $K_r$ 和反馈 $K_e$ 是离线计算得到的。

虽然 $η_k$ 和 $ζ_k$ 都用于保证系统约束可行性，但它们的调节机制不同。这可以通过观察 (4) 中的预测窗口来解释，因为 $η_k$ 在瞬态模式 $∀i∈ N[0,n_c-1]$ 中起作用，而 $ζ_k$ 主要影响终端模式 $∀i ≥ n_c$。



#### 问题求解

在本节中，我们将解释如何将 PR 和 RG 系统地集成到预测性 FTTC（3）和（4）中，以及如何确定相关的最优参数。为了更好地描述各部分的功能，我们从 FTTC（3）和（4）中提取了三种 FTTC 策略，即无约束鲁棒 FTTC、基于 PR 的鲁棒预测 FTTC 和基于 PR 和 RG 的鲁棒预测 FTTC。下文将逐步给出具体的分析和设计条件。

##### 无约束鲁棒 FTTC 设计

控制率 $u_k^{un}=K_e(x_k-x_{r,k})+u_{r,k}-\hat{f}_k$ 。

$\hat{f}_k$ 的估计通过公式 (1) 得到：$d_k=x_{k+1}-A(\theta_k)x_k-B(\theta_k)u_k=B(\theta_k)f_k+D(\theta_k)w_k$ ，

那么 $B(\theta_k)f_k=d_k-D(\theta_k)w_k$  ，可以得到：
$$
f_k=H(\theta_k)(d_k-D(\theta_k)w_k)\\
H(\theta_k)=(B^T(\theta_k)B(\theta_k))^{-1}B^T(\theta_k)
$$
$f_k$ 依赖下一时刻的状态和无法测量的扰动，所以可以把上一时刻的 $f_{k-1}$ 作为当前时刻的近似估计：

![02-04](.\image\02-04.png)

这种处理方式会带来一定的故障补偿误差。但可以通过假设 3 确定其边界：
$$
\begin{align}
e_{f,k}&=f_k-\hat{f}_k\\
	   &=f_k-(f_{k-1}+H(\theta_{k-1})D(\theta_{k-1})w_{k-1})\\
	   &=\delta_{f,k}-H(\theta_{k-1})D(\theta_{k-1})w_{k-1}\\
	   &\in F\oplus (-H(\theta_{k-1})D(\theta_{k-1}))W
\end{align}
$$
将集合再度进行放大（超立方体）：

![02-05](.\image\02-05.png)

$|| H(\theta_{k-1})D(\theta_{k-1})\leq\mathcal{H}\mathcal{1}_{nh}\mathcal{D} ||$ ，$e_{f,k}\in F\oplus(-\mathcal{H}\mathcal{1}_{nh}\mathcal{D}W)$  。



##### 预计调节鲁棒预测 FTTC 设计

根据上一节的无约束鲁棒 FTTC 设计，考虑preview regulator ，设计新的控制律：
$$
u_k^c=K_e(x_r-x_{r,k})+u_{r,k}-\hat{f}_k+c_k
$$
由于 $x_{r,k}=A(\theta_k)x_{r,k}+B(\theta_k)u_{r,k}$ ，所以：
$$
x_{k+1}-x_{r,k}=A(\theta_k)(x_k-x_{r,k})+B(\theta_k)(u_k^c+f_k-u_{r,k})+D(\theta_k)w_k
$$
代入 $u_k^c=K_e(x_r-x_{r,k})+u_{r,k}-\hat{f}_k+c_k$ ，$x_{r,k}=K_{xr}(\theta_k)r_{k+1}$ ，$c_k=K_c\vec{c}_k,r_{k+1}=C_r\vec{r}_{k+1}$：

![02-06](.\image\02-06.png)

 $c_k$ 是为了保证系统可解并满足约束，要满足这一点进行设计。

**定义1：** 自治不变集，当 $K_c\vec{c}_k=0$ 时，存在集合 $S_{RPI}$ 是不变集。$S_{MRPI}$ 是最大不变集，包含了 $S_{RPI}$ 。

**定义2：** 控制不变集，当 $K_c\vec{c}_k\neq0$ 时，存在集合 $S_{RCI}$ 是不变集，$S_{MRCI}$ 是最大不变集，包含了 $S_{RCI}$ 。
