### 建模

AUV 模型：

<img src=".\image\02.png" alt="02" style="zoom:50%;" />

以公式（3）中的 v 为状态，τ 为输入，得到非线性状态空间方程：

<img src=".\image\03.png" alt="03" style="zoom:50%;" />

在仿真程序中使用这个非线性方程来计算 v 的导数，与仿真程序中原来的计算方式对比，得到结果：

![01](E:\Library\硕士实验室\MPC\Note\2024_4\mpc-model\image\01.png)

可以看出非线性方程正确。



### MPC控制AUV

虽然能够得到状态 v 的非线性方程，但是还有未解决的问题：模型的控制输入 u 与 方程中的τ不相等，两者也存在非线性关系：τ 代表 AUV 在六个自由度所受的力矩，u 代表执行器的变化量。

<img src=".\image\04.png" alt="04" style="zoom:70%;" />

有两种方法解决这个问题：

1. 直接将 τ 和 u 的非线性对应关系置入状态空间方程中，直接对 u 进行优化
2. 不在 MPC 中优化 u，而是计算出 τ 以后再计算对应的 u 。



### MPC-based 3-D trajectory tracking for an autonomous underwater vehicle with constraints in complex ocean environments  

建立状态空间模型：

<img src="E:\Library\硕士实验室\MPC\Note\2024_4\mpc-model\image\05.png" alt="05" style="zoom:80%;" />

建立模型后，就可以使用 MPC 得到最优的模型输入 u （这里的 u 表示速度向量 v 的微分）。

然后可以根据这个结果，结合公式（3），得到 τ，再根据 τ 计算执行机构的动作。

这个方法的好处是：在计算 MPC 优化时，可以对状态空间方程进行线性化，用线性 MPC 来求解，可以提高求解速度，之所以可以进行线性化，文章的解释是：

<img src=".\image\06.png" alt="06" style="zoom:80%;" />



### Trajectory Tracking Control of an Autonomous Underwater Vehicle Using Lyapunov-Based Model Predictive Control  

该方法直接将 τ 和 u 的映射建模到状态空间方程：

<img src=".\image\07.png" alt="07" style="zoom:67%;" />

存疑。