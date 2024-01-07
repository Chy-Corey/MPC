### Fault-tolerant reference generation for model predictive control with active diagnosis of elevator jamming faults



#### Abstract

- 研究问题

  升降舵卡死故障情况下的纵向控制问题

- 控制策略

  使用可重构容错预测控制来解决永久和临时的执行器卡死故障

- 特点

  MPC 作为容错控制器可以帮助故障检测模块区分永久和临时的卡斯故障。

  每次检测到故障时，故障检测模块命令 MPC 执行预定义的重构序列来诊断故障的根本原因。

  一个人工参考信号，能够反映执行器工作范围，被用来指导系统泰国这个信号序列来重构。



