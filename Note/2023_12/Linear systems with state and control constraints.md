## Linear systems with state and control constraints The theory and application of maximal output admissible sets.

定义不变集：
$$
O_{\infty}(A,C,Y)=\{x\in R^n:CA^tx\in Y,\forall t\geq 0 \}
$$
**定理 2.1：**

1. $O_{\infty}$ 继承了 $Y$ 的以下属性：闭合，凸，对称
2. 如果 C，A 可观且 Y 有界，则 $O_{\infty}$ 有界。
3. 如果 A 李雅普诺夫稳定，且 Y 包括原点，那么 $O_{\infty}$ 包含原点。
4. $O_{\infty}(A,C,\alpha Y)=\alpha O_{\infty}(A,C,Y)$

证明：

2. 可观性矩阵：$H=[C^T\ \ \ A^TC^T\ \ \ \cdots\ \ \ (A^T)^{n-1}C^T]^T$ 满秩。$Hx=z \Rightarrow x=H^{\dagger}z$ 。

   那么 $O_{\infty}\subset O_{N-1}=H^{\dagger}Y$ ，由于 Y 有界，那么不变集有界。

3. 定义 $S(r)=\{x:||x||<r\}$ ，由于 A 李雅普诺夫稳定，可定义范数上界：$||CA^tx||\leq \gamma_1||x||$ 。选择 $\gamma_2$ 满足 $S(\gamma_2)\subset Y，\gamma_1||x||\leq\gamma_2$ ，那么 $CA^tx\in Y$ ，满足了 $O_{\infty}$ ，因此 $O_{\infty} \supset \{x\in R^n:\gamma_1||x||\leq\gamma_2\}=S(\gamma_2/\gamma_1)$ 
4. $O_{\infty}(A,C,\alpha Y) = \{x:CA^t\alpha^{-1}x\in Y \}=\{\alpha(\alpha^{-1}x):CA^t\alpha^{-1}x\in Y\}$



**Remark 2.1 & 定理 2.2：**

<img src="E:\Note\MPC\Note\2023_12\image\20.png" alt="20" style="zoom:60%;" />

1. $O(A,C,Y)$ 满足以下性质：$O_{\infty}\subset O_{t_2} \subset O_{t_1},t_1\leq t_2$

   若存在一个t，使得 $O_{\infty}=O_{t}$ , 称 $O_{\infty}$ 是**finitely determined(有限确定的**）。假设t是保证上式成立则最小的t，称t为**output admissibility index(输出容许索引）** 。

2. 当且仅当存在一个t使得 $O_{t+1}=O_{t}$ , $O$是有限确定的。

有限确定的最大容许集一般比非有限确定的最大容许集有更简单的形式，而且，可以通过有限递归过程获得。



**定理 2.3：**

考虑约束Y是由一系列的不等式约束得到的： $Y=\{f_i(y)\leq 0\}$ ,那么在什么条件下，O可以被类似的定义呢？

假设 A 李雅普诺夫稳定，那么如果满足：

1. 定义函数 $g_i(x)=sup\{f_i(CA^tX):t\geq 0\}$ （sup即上确界）
2. O 包含原点
3. $O_{\infty}=\{x\in R^n:g_i(x)\leq 0 \}$

那么不变集可表示为：
$$
O_{\infty}=\{x\in R^n:f_i(CA^tX)\leq 0 \}
$$
这里有 $s\times (t+1)$ 个不等式构成约束（s 是 y 的阶数），由于 t 可取到无穷，所以约束方程是无穷个。

然而并非中的所有不等式约束都有效。有效的不等式具有特殊的结构，在定理2.4中给出。



**定理 2.4：**

对于定理 2.3 给出的 $O_{\infty}$ ，存在非空整数集 $S^*\in \{1,2,\dots,s \}$ 和 $t_i^*,i\in S^*$ ，使：
$$
O_{\infty} = \{x\in R^n:f_i(CA^tx)\leq0,i\in S^*,t\in \{0,\dots,t_i^* \} \}
$$
对 $S^*$ 中的所有 i 和任意 t，存在 $x\in O_{\infty}$ 使得 $f_i(CA^ix)=0$ 。



**定理 4.1：**

若

1. A渐近稳定，
2. (A, C)可观，
3. Y有界，
4. Y包含原点

则O可以被有限确定

证明：

根据定理 2.1 ，由于假设 2、3，$O_{\infty}$ 有界，那么存在 r>0 ，使 $O(t)\subset S(r),t\geq n-1$ 。又根据假设 1 ，当 t 趋向于无穷时，$CA^t \rightarrow 0$ ，所以随着整数 k 的增大，$CA^{k+1}$ 减小。则一定存在 $k\geq n-1$ ，使 $CA^{K+1}S(r)\subset Y$。从而有 $CA^{K+1}O_k\subset Y$ 。因此有 $O_{k+1}=O_k$ 。



**定理 4.2：** 

接下来讨论对于A满足李雅普诺夫稳定的系统如何得到有限确定的 O 。

对于李雅普诺夫稳定的系统：
$$
C=\begin{bmatrix}
C_L&C_S
\end{bmatrix},
A=\begin{bmatrix}
I_d&0\\0&A_s
\end{bmatrix}
$$
定义：
$$
\hat{C}=\begin{bmatrix}
C_L&0\\C_L&C_S
\end{bmatrix},

X_L=\{x:[C_L\ \ \ 0]x\in Y \}
$$
那么：

1. $O_{\infty}\subset X_L$
2. $O_{\infty}(A,C,Y)=O_{\infty}(A,\hat{C},Y\times Y)$

证明：

假设存在 $x\in O_{\infty}(A,C,Y)$ ，并且 $C_Lx\notin Y$ ，把 $O_{\infty}(A,C,Y)$ 中的元素写为表达式：$[C_L\ \ \ 0]x+[0\ \ \ C_s\ \ \ A_s^t]x$ 。由于 $C_Lx\notin Y$ 且 Y 封闭、包含原点，所以 $C_Lx+w$ 也不在集合 Y 中，但 $[C_L\ \ \ 0]x+[0\ \ \ C_s\ \ \ A_s^t]x\in Y$ ，矛盾，所以 $O_{\infty}\subset X_L$ 成立。
$$
O_{\infty}(A,\hat{C},Y\times Y)=\\ \begin{align} 
1.&\ [C_L\ \ \ 0]x\in Y\\
2.&\ [C_L\ \ \ C_s]A^tx\in Y
\end{align}
$$
两集合求交集。

第一个集合包含 $O_{\infty}(A,C,Y)$ ，第二个集合就是 $O_{\infty}(A,C,Y)$ ，所以$O_{\infty}(A,C,Y)=O_{\infty}(A,\hat{C},Y\times Y)$ 。

通过定理 4.2 ，如果 $O_{\infty}(A,C,Y)$ 不可以有限决定，可以转换为 $O_{\infty}(A,\hat{C},Y\times Y)$ 的形式。先通过第一个集合 $[C_L\ \ \ 0]x\in Y$  来逼近特征值等于 1 的变量，再求解优化问题。

但是这种方法可能会导致逼近到边界时，约束条件剧增（Fig 2）。



**定理 5.1：**