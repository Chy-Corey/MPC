### Fault-tolerant tracking control optimization of constrained LPV systems based on embedded preview regulation and reference governance



#### abstract

本文通过嵌入**优化预览调节（optimal preview regulation）**和**参考治理（reference governance）**，对相关约束预测容错跟踪控制（fault-tolerant tracking control, FTTC）方法提出了一些新的改进。**这种策略的主要新颖之处在于可以充分调度有限未来参考的一些有价值信息，从而优化鲁棒跟踪性能并显著扩大容错容许区域。**

为了更好地说明所提策略的广泛适用性，我们考虑了一类具有状态/输入约束的 LPV 系统的鲁棒 FTTC 问题。总的来说，所涉及的关键设计包括三个部分。

1. 首先，通过结合跟踪误差反馈、参考输入调节和故障信号补偿，构建了无约束 FTTC 组件。它用于保证闭环系统在约束条件未被激活时的鲁棒跟踪稳定性。
2. 其次，设计了带有嵌入式最优预览调节器的管式预测 FTTC 策略，以实现稳健的约束满足和瞬态响应改善。
3. 第三，额外集成了一个嵌入式参考调节器，以显著扩大容错可容许区域的大小。这一设计进一步加强了约束 FTTC 优化的可行性。这些结果的有效性最终通过单晶体管直流/直流正向转换器的案例研究得到了验证。



#### 问题描述



