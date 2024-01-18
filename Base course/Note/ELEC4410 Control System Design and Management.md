# Control System Design and Management

## Lec 02

- 线性时不变系统的状态空间方程

### 一、线性化

定义标称变量：$\tilde{x}(t),\tilde{x}_0,\tilde{u}(t)$，那么：
$$
\dot{\tilde{x}}(t)+\dot{\tilde{x}}_{\delta}(t)=f(\tilde{x}(t)+\tilde{x}_{\delta}(t), \tilde{u}(t)+\tilde{u}_{\delta}(t),t)
$$
$\dot{\tilde{x}}(t)+\dot{\tilde{x}}_{\delta}(t)$ 是状态，$\dot{\tilde{x}}_{\delta}(t)$ 是状态距离标称变量的距离。

$f$ 是 $n$ 个函数组成的函数向量，对 $f_i$ 泰勒一阶展开：
$$
{f_{i}(\tilde{x}+x_{\delta},\tilde{u}+u_{\delta},\,t)\approx f_{i}(\tilde{x},\tilde{u},\,t)+\frac{\partial f_{i}}{\partial{x_{1}}}(\tilde{x},\tilde{u},t){x_{\delta 1}}+\cdots+\frac{\partial f_{i}}{\partial{x_n}}(\tilde{x},\tilde{u},t){x_{\delta n}}}\\
+\frac{\partial f_{i}}{\partial{u_{1}}}(\tilde{x},\tilde{u},t){u_{\delta 1}}+\cdots+\frac{\partial f_{i}}{\partial{u_m}}(\tilde{x},\tilde{u},t){u_{\delta m}}
$$
那么，状态空间方程可以写成矩阵形式：
$$
\dot{\tilde{x}}(t)+\dot{\tilde{x}}_{\delta}(t)=f(\tilde{x}(t), \tilde{u}(t),t)+\frac{\partial{f}}{\partial{x}}(\tilde{x},\tilde{u},t)x_{\delta}+\frac{\partial{f}}{\partial{u}}(\tilde{x},\tilde{u},t)u_{\delta}
$$

$$
\frac{\partial f}{\partial x}\triangleq\begin{bmatrix}\frac{\partial f_1}{\partial x_1}&\frac{\partial f_1}{\partial x_2}&\cdots&\frac{\partial f_1}{\partial x_n}\\\frac{\partial f_2}{\partial x_1}&\frac{\partial f_2}{\partial x_2}&\cdots&\frac{\partial f_2}{\partial x_n}\\\cdots&\cdots&\cdots&\cdots\\\frac{\partial f_n}{\partial x_1}&\frac{\partial f_n}{\partial x_2}&\cdots&\frac{\partial f_n}{\partial x_n}\end{bmatrix}
$$

由于 $\dot{\tilde{x}}(t)=f(\tilde{x}(t), \tilde{u}(t),t)$ ，所以：
$$
\dot{\tilde{x}}_{\delta}(t)=A(t)x_{\delta}(t)+B(t)u_{\delta}(t)\\
A(t) =\frac{\partial{f}}{\partial{x}}(\tilde{x},\tilde{u},t)\\
B(t) =\frac{\partial{f}}{\partial{u}}(\tilde{x},\tilde{u},t)
$$
如果选择的标称点 $\tilde{x}(t)$ 和 $\tilde{u}(t)$ 是稳定的，即 $\dot{\tilde{x}}(t)=0$ ，那么在这个稳定点附近的线性化方程如上所示。

