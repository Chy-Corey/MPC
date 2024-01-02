## Linear systems with state and control constraints The theory and application of maximal output admissible sets.

定义不变集：
$$
O_{\infty}(A,C,Y)=\{x\in R^n:CA^tx\in Y,\forall t\geq 0 \}
$$
定理 2.1：

1. $O_{\infty}$ 继承了 $Y$ 的以下属性：闭合，凸，对称
2. 如果 C，A 可观且 Y 有界，则 $O_{\infty}$ 有界。
3. 如果 A 李雅普诺夫稳定，且 Y 包括原点，那么 $O_{\infty}$ 包含原点。
4. $O_{\infty}(A,C,\alpha Y)=\alpha O_{\infty}(A,C,Y)$

证明：

2. 可观性矩阵：$H=[C^T\ \ \ A^TC^T\ \ \ \cdots\ \ \ (A^T)^{n-1}C^T]^T$ 满秩。$Hx=z \Rightarrow x=H^{\dagger}z$ 。

   那么 $O_{\infty}\subset O_{N-1}=H^{\dagger}Y$ ，由于 Y 有界，那么不变集有界。

3. 定义 $S(r)=\{x:||x||<r\}$ ，由于 A 李雅普诺夫稳定，可定义范数上界：$||CA^tx||\leq \gamma_1||x||$ 。选择 $\gamma_2$ 满足 $S(\gamma_2)\subset Y，\gamma_1||x||\leq\gamma_2$ ，那么 $CA^tx\in Y$ ，因此 $(\gamma_2/\gamma_1)\geq ||x|| \Rightarrow $ ? 
4. $O_{\infty}(A,C,\alpha Y) = \{x:CA^t\alpha^{-1}x\in Y \}=\{\alpha(\alpha^{-1}x):CA^t\alpha^{-1}x\in Y\}$