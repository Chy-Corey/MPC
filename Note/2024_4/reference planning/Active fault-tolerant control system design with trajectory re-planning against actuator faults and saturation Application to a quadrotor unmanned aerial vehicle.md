## Active fault-tolerant control system design with trajectory re-planning against actuator faults and saturation  



在过去的 30 年中，人们开发了各种容错控制（FTC）方法，用于解决各种系统的执行器或组件故障，无论是否具有跟踪控制目标。然而，很少有 FTC 策略能**在故障发生后跟踪的参考轨迹与系统剩余资源之间建立联系**。这是一个尚未解决的问题，在文献中没有得到很好的考虑。

本文的主要贡献在于设计了一种可重构的 FTC 和轨迹规划方案，重点是利用**微分平坦**进行在线决策。在无故障情况下，根据可用的执行器资源，合成参考轨迹，以便在不违反系统约束条件的情况下，尽可能快地驱动系统达到所需的设定点。在故障情况下，所提出的主动式 FTC 系统（AFTCS）包括在使用基于参数估计的**非特征卡尔曼滤波器**的故障检测和诊断方案诊断出执行器故障后，合成一个可重新配置的反馈控制和修改后的参考轨迹。利用平整度概念和基于补偿的可重构控制器对轨迹重新规划进行整合，可有效处理 AFTCS 设计中的致动器故障和饱和问题。




