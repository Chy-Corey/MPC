

### Fault-tolerant reference generation for model predictive control with active diagnosis of elevator jamming faults  

文章考虑了两种故障：

![01](E:\Library\硕士实验室\MPC\Note\2024_6\ftmpc\img\01.png)

设计了对于这两种故障的辨识方法，容错方法只有一种，不针对故障区分。

容错MPC：

![02](E:\Library\硕士实验室\MPC\Note\2024_6\ftmpc\img\02.png)

与 Limon 的方法相同：

![03](E:\Library\硕士实验室\MPC\Note\2024_6\ftmpc\img\03.png)

文章中的约束 (7 e) 对应 Limon 的 $(x(N), \theta)\in X_f^w$ 。







### An Active Fault-Tolerant MPC for Systems with Partial Actuator Failures

![05](E:\Library\硕士实验室\MPC\Note\2024_6\ftmpc\img\05.png)

线性系统：

![04](E:\Library\硕士实验室\MPC\Note\2024_6\ftmpc\img\04.png)

有三种执行器故障：partial actuator failure, the actuator outage and the actuator stuck（部分执行器故障，断电，卡死）。

文章考虑第一种故障：

![06](E:\Library\硕士实验室\MPC\Note\2024_6\ftmpc\img\06.png)

**$F_k$ 是不知道的。**

对于故障后系统：

![07](E:\Library\硕士实验室\MPC\Note\2024_6\ftmpc\img\07.png)

执行器输入 $u_k$ 和故障 $F_k$ 耦合，需要将其分开，以观测器的形式辨识故障的数值。

![08](E:\Library\硕士实验室\MPC\Note\2024_6\ftmpc\img\08.png)

设计最小方差观测器：

![09](E:\Library\硕士实验室\MPC\Note\2024_6\ftmpc\img\09.png)

文章的MPC使用两层控制。

第一层：

![10](E:\Library\硕士实验室\MPC\Note\2024_6\ftmpc\img\10.png)

如果无法满足等式约束，可以使用：

![11](E:\Library\硕士实验室\MPC\Note\2024_6\ftmpc\img\11.png)

第二层：

![12](E:\Library\硕士实验室\MPC\Note\2024_6\ftmpc\img\12.png)



这篇文章的优点是做了一整套工作：从故障诊断到容错控制，考虑的是执行器效率降低的故障，能够实现故障情况下的无偏跟踪。

这种方法没有优化故障情况下的动态性能。



### Reconfigurable Fault Tolerant Flight Control based on Nonlinear ModelPredictive Control

NPMC 与LMPC的性能对比：`In [8], NMPC is shown to providebetter performance compared to a Linear Parameter Varying(LPV) control approach for the F-16 model used in the workpresented here. `

![14](E:\Library\硕士实验室\MPC\Note\2024_6\ftmpc\img\14.png)

- case 1：根据 FDD 的信息更新约束，不重构。
- case 2：没有 FDD ，根据内环的故障检测和重构能力
- case 3：根据 FDD 进行重构。