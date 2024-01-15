### Fault-tolerant tracking control optimization of constrained LPV systems based on embedded preview regulation and reference governance



#### abstract

本文通过嵌入**优化预览调节（optimal preview regulation）**和**参考治理（reference governance）**，对相关约束预测容错跟踪控制（fault-tolerant tracking control, FTTC）方法提出了一些新的改进。**这种策略的主要新颖之处在于可以充分调度有限未来参考的一些有价值信息，从而优化鲁棒跟踪性能并显著扩大容错容许区域。**

为了更好地说明所提策略的广泛适用性，我们考虑了一类具有状态/输入约束的 LPV 系统的鲁棒 FTTC 问题。总的来说，所涉及的关键设计包括三个部分。

1. 首先，通过结合跟踪误差反馈、参考输入调节和故障信号补偿，构建了无约束 FTTC 组件。它用于保证闭环系统在约束条件未被激活时的鲁棒跟踪稳定性。
2. 其次，设计了带有嵌入式最优预览调节器的管式预测 FTTC 策略，以实现稳健的约束满足和瞬态响应改善。
3. 第三，额外集成了一个嵌入式参考调节器，以显著扩大容错可容许区域的大小。这一设计进一步加强了约束 FTTC 优化的可行性。这些结果的有效性最终通过单晶体管直流/直流正向转换器的案例研究得到了验证。



#### 问题描述

线性变参数系统：

![2-1](E:\Library\硕士实验室\MPC\Note\2024_1\image\2-1.png)

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
&J_k=\Sigma_{t=0}^{\infty}U(x_{k+t},u_{k+t})\\
& U(x_{k+t},u_{k+t})=(x_{k+t}-x_{r,k+t})^TQ(x_{k+t}-x_{r,k+t})+(u_{k+t}-u_{r,k+t})^TR(u_{k+t}-u_{r,k+t})
\end{align}
$$
