## SVD & 矩阵子空间

> Hongyu Chen
>
> 2314727047@qq.com



### 一、子空间

对矩阵 $A\in R^{m\times n}$，其子空间定义如下。

#### 1. 像空间 & 列空间

$$
R(A)=\{Ax|x\in R^n\}\subseteq R^m
$$

像空间是向量 $x\in R ^n$ 经矩阵 $A$ 变换后所有像的集合，也可以将乘积 $Ax$ 看成关于 $A$ 的列向量的线性组合：
$$
A = [A_1\ \ A_2\ \ \cdots \ \ A_n]\\
Ax =\Sigma_{i=1}^{n}x_iA_i
$$
所以像空间也可以称为列空间。

#### 2. 行空间

即 $A$ 的行向量的线性组合，可以用 $R(A^T)$ 表示。

#### 3. 零空间

$$
N(A)=\{x|Ax=0\}\subseteq R^n
$$

零空间是 $Ax=0$ 所有解的集合。

#### 4. 左零空间

$$
N(A^T)=\{y|A^Ty=0\}\subseteq R^m
$$

左零空间是 $y^TA=0^T$ 所有解的集合。



### 二、SVD

对矩阵 $A\in R^{m\times n}$，$rank(A)=r$ ，SVD分解为 $A=U\Sigma V^H$ ，$U\in R^{m\times m}$ ，$\Sigma \in R^{m\times n}$ ，$V\in R^{n\times n}$ 。

将 $U$ 和 $V$ 按列分块：
$$
U_1=[u_1,\cdots,u_r],\ \ U2=[u_{r+1},\cdots,u_m]\\
V_1=[v_1,\cdots,v_r],\ \ V2=[v_{r+1},\cdots,v_n]
$$
将矩阵形式 $U\Sigma V^H$ 展开，得到 SVD 的展开形式：
$$
A=\sigma_1u_1v_1^H+\sigma_2u_2v_2^H+\cdots+\sigma_ru_rv_r^H
$$


#### 1. 像空间

对于 $y=Ax$ ，$x\in R^n$ ，将 $A$ 写为 SVD 展开形式：
$$
\begin{eqnarray}   
y&=&Ax     \\
~&=&\Sigma_{i=1}^r \sigma_iu_iv_i^Hx   \\
~&=&\Sigma_{i=1}^r (\sigma_iv_i^Hx)u_i \\
~&=&\Sigma_{i=1}^r \alpha_i u_i
\end{eqnarray}
$$
上式 $v_i^Hx$ 能和 $u_i$ 调换顺序，是因为 $v_i^Hx$ 是标量。

由于 $U$ 和 $V$ 都是酉矩阵（对应实数域的正交矩阵），所以 $\alpha_i$ 各标量之间相互独立。那么矩阵 $A$ 的像空间（列空间）：
$$
R(A)=\{y|y=Ax,\forall x\in R^n\}=R(U_1)
$$

#### 2. 零空间

与像空间类似，令 $Ax=0$ ：
$$
\begin{eqnarray}   
Ax&=&\Sigma_{i=1}^r \sigma_iu_iv_i^Hx     \\
~&=&\Sigma_{i=1}^r (\sigma_iv_i^Hx)u_i \\
~&=&\Sigma_{i=1}^r \alpha_i u_i        \\
~&=&0
\end{eqnarray}
$$
要得到 $Ax=0$ ，就要让 $\alpha_i = 0$ ，由于 $V$ 是酉矩阵，所以当 $\alpha_i = 0$ 时，$x$ 是 $v_{r+1},\cdots,v_n$ 的线性组合。所以：
$$
N(A)=\{x|Ax=0\}=R(V_2)
$$

#### 3. 行空间和左零空间

可以对 $A^T$ 进行 SVD 分解：
$$
A=U\Sigma V^H \\
A^T=V\Sigma^HU^H
$$
那么：
$$
R(A^T) = R(V_1)\\
N(A^T) = R(U_2)
$$

#### 4. 总结

![svd & 子空间](.\img\svd & 子空间.png)



### 三、举例

求解方程组：
$$
\begin{bmatrix}
A-I&B&0\\
C&D&-I
\end{bmatrix}\begin{bmatrix}
x_s\\
u_s\\
y_t
\end{bmatrix}=\begin{bmatrix}
0\\
0\\
\end{bmatrix}
$$
令 $z_s=[x_s^T\ \ \ \ u_s^T]^T$ ，方程组 $[A-I\ \ \ \ B]\ z_s=0$ 的解为 $[A-I\ \ \ \ B]$ 的零空间，对其SVD分解后就是 $R(V_2)$ 。直接令 $M_\theta = V_2$ ，参数 $\theta \in R^{n_{\theta}}$ ，那么 $R(V_2)$ 可以表示为 $M_{\theta}\theta$ ，即 $z_s=M_{\theta}\theta$ 。

那么 $y_t=[C\ \ \ \ D]\ z_s=[C\ \ \ \ D]\ M_{\theta}\theta$ ，令 $N_\theta=[C\ \ \ \ D]\ M_{\theta}$  ，即可表示为：
$$
z_s=M_\theta \theta\\
y_t=N_\theta \theta
$$

