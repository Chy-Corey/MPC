### Can reference governors outperform tube-based predictive control



本文比较了参考调节器方案和 tube-based MPC 算法，以解决具有输出干扰的简单跟踪问题。主要目的是考虑在这种情况下，在多大程度上值得采用 tube-based MPC 的方法，或者是否可以采用一种简单得多的方法来获得同样的可行性保证和良好性能。本文对两种方法的主要特点、性能和一般设置进行了描述和比较。对两种算法的跟踪性能进行了分析。



#### 问题设置和初步结论

##### 系统、模型和假设

<img src=".\image\04-01.png" alt="04-01" style="zoom:67%;" />

稳态方程（考虑输出扰动）：

<img src=".\image\04-02.png" alt="04-02" style="zoom:67%;" />

将公式 (4) 转化为：

<img src=".\image\04-03.png" alt="04-03" style="zoom:67%;" />

控制律可以表示为：

<img src=".\image\04-04.png" alt="04-04" style="zoom:67%;" />

##### 跟踪 MPC

<img src=".\image\04-05.png" alt="04-05" style="zoom:67%;" />

使用预先设计的控制律：

<img src=".\image\04-06.png" alt="04-06" style="zoom:67%;" />



#### Tube MPC

管道概念的核心是扰动不变集[11]，即在有界扰动的情况下，从原点出发可到达的状态集。状态、输入和局部轨迹的参数化及其各自的 "管道 "可在下文中描述。由于 (1) 中模型的不确定性，预测状态由一个包含一系列集合 {X0,X1,...} 的管道来描述。每个集合都包含了未来不确定性的所有实现情况下，特定未来时间瞬间的可能状态：