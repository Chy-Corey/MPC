## Kalman Filter



考虑扰动：
$$
\begin{equation}
\begin{aligned}
x(k+1) &= Ax(k)+Bu(k)+w(k)\\
y(k)&=Cx(k)+v(k)
\end{aligned}\tag{1}
\end{equation}
$$
两个扰动满足正态分布：$p(w)\in N(0, Q)$ ，$p(v)\in N(0, R)$ 。

定义：

- $x_k$ ：真实状态值
- $\hat{x}_k^-$ ：先验状态估计值，可以理解为状态的预测值
- $\hat{x}_k$ ：后验状态估计值，根据当前时刻的数值得到的经验

两个估计值对应的误差：
$$
e_k^-=x_k-\hat{x}_k^-\\
e_k=x_k-\hat{x}_k
$$
两者的协方差：
$$
P_k^-=E[e_k^-e_k^{-T}]\\
P_k=E[e_ke_k^{T}]
$$
先验状态估计值的表达式可以直接给出：
$$
\hat{x}_k^-=A\hat{x}_k+Bu_k
$$
后验状态估计值的表达式通过线性反馈的形式定义：
$$
\hat{x}_k=\hat{x}_k^-+K(y_k-C\hat{x}_k^-)\tag{2}
$$
$K$ 的取值大小反映了更相信测量值还是预测值：假设 $y_k=x_k+v$ ，$C=I$ ，当 $K=0$ 时，后验状态估计值完全取决于预测值；当 $K=I$ 时，后验状态估计值完全取决于测量值。

联合公式 (1) 和 (2)：
$$
\hat{x}_k=\hat{x}_k^-+K(Cx_k+v_k-C\hat{x}_k^-)\\
\hat{x}_k=\hat{x}_k^-+KCx_k+Kv_k-KC\hat{x}_k^-\\
\Rightarrow \hat{x}_k-x_k=\hat{x}_k^--x_k+KC(x_k-\hat{x}_k^-)+Kv_k\\
\Rightarrow e_k=(I-KC)e_k^--Kv_k
$$
根据此结果，代入协方差公式：
$$
\begin{align}
P_k &=E[e_ke_k^{T}]\\
    &=E[((I-KC)e_k^--Kv_k)(e_k^{-T}(I-KC)^T-v_k^TK^T)]\\
    &=(I-KC)E[e_k^-e_k^{-T}](I-KC)^T+KE[v_kv_k^T]K^T
\end{align}
$$
这里展开省略了两个交叉项，是因为互相独立，协方差为 0。

由于 $E[e_k^-e_k^{-T}]=P_k^-$ ，$E[v_kv_k^T]=R$ ，所以上式可继续化简：
$$
\begin{align}
P_k &=(I-KC)E[e_k^-e_k^{-T}](I-KC)^T+KE[v_kv_k^T]K^T\\
    &=(I-KC)P_k^-(I-KC)^T+KRK^T\\
    &=P_k^--KCP_k^--P_k^-C^TK^T+K(CP_k^-C^T+R)K^T\tag{3}
\end{align}
$$
将 $P_k$ 对 $K$ 求导，使导数等于 0 ，得到方差最小的 Kalman 增益：
$$
\begin{align}
\frac{\partial{P_k}}{\partial{K}}&=-2(P_k^-C^T)+2K(CP_k^-C^T+R)=0\\
K&=P_k^-C^T(CP_k^-C^T+R)^{-1}
\end{align}
$$
将 K 的表达式代回到 (3) 中：
$$
P_k=(I-KC)P_k^-
$$
还需要知道 $P_k^-$ 如何表达，与 $P_k$ 类似，也是将定义式展开：
$$
\begin{align}
P_{k+1}^- &= E[e_{k+1}^-e_{k+1}^{-T}]=E[(Ae_k+w_k)(Ae_k+w_k)^T]\\
		  &= E[(Ae_k)(Ae_k)^T]+E[w_kw_k^T]\\
		  &= AP_kA^T+Q
\end{align}
$$
所以 Kalman 滤波器的反馈率计算可以总结为以下三个公式：
$$
\begin{align}
&K=P_k^-C^T(CP_k^-C^T+R)^{-1}\\
&P_{k+1}^- = AP_kA^T+Q\\
&P_k=(I-KC)P_k^-
\end{align}
$$
