### Optimal robust trajectory tracking control of a X-rudder AUV with velocity sensor failures and uncertainties

>1. 本文通过观测器来估计扰动，从而能够抵消速度传感器故障，具有鲁棒性。
>2. 本文的纵向面建模可供参考。
>3. reference governor，Peng 等人（2019）。
>







#### Abstract

本文提出了一种最优鲁棒控制方法，用于受**速度传感器故障**和不确定性影响的X舵自主潜水器（AUV）的轨迹跟踪。设计了两个降阶扩展状态观测器（extended state observer, ESO）来估计**涌浪和波浪速度**，并在控制器设计中使用估计值替代所有线性速度相关参数，这有助于释放对线性速度测量的要求，并使控制器对线性速度传感器故障具有鲁棒性。在运动学控制环中，采用了视线（line-of-sight, LOS）制导法则和基于 Lyapunov 的控制，并根据估计的线速度计算未知攻击角。在动力学控制环中，利用扰动观测器和改进的终端滑模控制，构建了稳健的扰动抑制控制律。此外，还提出了一种多目标优化方法来实现 X-舵分配，该方法不仅节能，而且对舵故障具有鲁棒性，同时有助于解决**舵输入饱和**问题。最后，通过数值模拟比较，证明了所提方法的稳健性和有效性。



#### Introduction

由于陆地资源的枯竭，海洋开发受到越来越多的关注。过去几十年来，各种水下设备得到了发展。自主潜水器是应用最广泛的水下设备之一，在各种水下应用中发挥着越来越重要的作用，如：

1. 水下干预、监测和检查
2. 目标跟踪
3. 海洋调查

为了更好地完成上述水下任务，采用了以下两种基本思路来提高潜水器的性能
1) 设计有效的控制系统，以便在恶劣的海洋环境中准确可靠地操纵自动潜航器；
1) 设计新的执行器，如方向舵、推进器等，以提高潜航器的能力。

考虑到第二种情况，各种舵被应用于 AUV，以提高其机动性，如十字舵、H 型舵、**X 型舵**等。

与传统的交叉舵 AUV 相比，X-舵 AUV 的四个舵可以独立操作，因此具有更好的机动性和灵活性。然而，与交叉舵 AUV 相比，X-舵 AUV 的控制更为复杂，在现有文献中研究较少。因此，本文将讨论 X舵 AUV 的轨迹跟踪控制。



Qiao 等人（2017）提出了三种指数收敛鲁棒控制器，即最小最大型控制器、饱和型控制器和平滑过渡型控制器，用于驱动 AUV 跟踪预定轨迹。von Ellenrieder（2019）利用扰动观测器和非线性动态表面控制，为存在未知时变扰动、输入饱和和致动器速率限制的海洋航行器的轨迹跟踪提出了一种 n 自由度非线性控制法。Zheng和Sun（2016）为无人驾驶MSV提出了自适应ILOS制导法，其中引入了RBFNN来处理时变洋流。Yu 等人（2018）建立了视线（LOS）制导法，用于在传感器噪声和洋流存在的情况下，通过 AUV 对海底电缆进行鲁棒磁力跟踪。Miao 等人（2017）提出了一种用于 AUV 路径跟踪控制的改进型复合视线（CLOS）制导法则，它可以估计未知侧滑角并补偿时变洋流的影响。Chen 等人（2019）提出了一种具有瞬态性能保证的自适应轨迹跟踪控制算法，该算法采用神经网络来逼近未知的外部干扰和不确定的流体力学。

上述大多数研究都基于这样一个假设，即 AUV 的所有**运动状态都是可测量的**。然而，考虑到复杂的水下环境，AUV 可能会出现**传感器故障**。在这种情况下，传统的控制方法可能无法奏效。基于状态观测器的控制方法可以为这种情况提供有效的解决方案。Yu 等人（2019b）提出了一种改进的基于扩展状态观测器的视线（ELOS）制导法，该制导法估计了涌浪和摇摆的线速度，并对未测量的侧滑角进行了补偿，但估计的速度**仅用于运动学控制，未应用于动力学控制**。Peng 和 Wang（2018）提出了一种输出反馈路径跟踪控制方法，适用于在垂直面内运动的欠驱动自主水下航行器，无需使用涌浪、翻滚和俯仰速度。Li 等人（2019）针对刚性航天器姿态控制系统提出了一种**基于有限时间扩展状态观测器的容错输出反馈控制，无需角速度测量，可在存在外部干扰和致动器故障的情况下进行控制。**在 Liu 等人（2019）的研究中，解决了无人水面飞行器在存在未知扰动以及未测量的激波、摇摆和偏航速度时的状态恢复和扰动估计问题。受现有方法的启发，本文提出了扩展状态观测器（ESO）来估计涌浪和波浪速度，并在控制器设计中使用估计值替代所有与速度相关的参数。因此，无需线性速度测量即可完成轨迹跟踪任务。

复杂的未知**外部干扰**是轨迹跟踪的一个重要问题，它可能影响模型精度并降低闭环系统性能（Wang 等人，2013 年，2018 年）。为解决这一问题，控制系统应具有良好的鲁棒性。常用的鲁棒控制方法包括但不限于滑模控制（Guo 等，2019）、H-无限控制（Zhang 等，2018b）、**模型预测控制（Li 和 Yan，2017）**、神经网络控制（Xia 等，2019b）、最优控制（Li 等，2018）、多代理（Xiao 等，2018）等。在所有鲁棒控制方法中，滑模控制是一种应用广泛的鲁棒控制技术，由于其对不确定性和干扰的鲁棒性，表现出了优异的跟踪性能（Van，2019）。Qiao 和 Zhang（2017）提出了一种自适应非矢量积分终端滑模控制（ANITSMC）方案，用于具有动力学不确定性和时变外部扰动的自主水下航行器（AUV）的轨迹跟踪。Xu 等人（2015）结合滑模控制和反步进技术，为具有参数不确定性和外部扰动的 AUV 设计了鲁棒轨迹跟踪控制器。Qiao 和 Zhang（2019）提出了一种自适应二阶快速非奇异终端滑模控制（ASOFNTSMC）方案，用于存在动态不确定性和时变外部扰动的全致动自主潜水器（AUV）的轨迹跟踪。然而，滑模控制经常会遇到颤振问题。为了保证控制的鲁棒性并避免颤振，本文基于扰动观测器和改进的终端滑模控制，构建了鲁棒扰动抑制控制律。

执行器动态是轨迹跟踪控制的另一项技术挑战。通常考虑的**执行器动态包括输入饱和、输入延迟**等，它们可能会影响控制系统的稳定性，甚至使整个系统变得不稳定。近年来，很多人都在努力解决这些问题。Cui 等人（2016）提出了一种抗饱和补偿器，以解决受制于致动器饱和的姿态控制问题。Chu 等人（2018）提出了新颖的辅助系统来处理输入饱和问题，并在此基础上开发了一种用于 AUV 潜水控制的自适应模糊滑模控制器。为了消除控制输入的非线性，Cui 等人（2017）利用强化学习为 AUV 提出了自适应神经网络控制。**Peng 等人（2019）利用参考管理器在状态和输入约束条件下生成最优参考信号，并在此基础上桥接参考环和控制环。**Sarhadi 等人（2016 年）针对存在输入饱和的 AUV 的俯仰和偏航通道提出了一种自适应自动驾驶仪，它采用了带有积分状态反馈和防风补偿器的模型参考自适应控制。然而，上述研究大多是针对交叉舵 AUV 设计的，无法解决 X 舵 AUV 面临的单输入多输出（SIMO）舵角分布问题。为解决这一问题，本文提出了一种多目标优化方法，该方法考虑了扭矩分配、能耗、输入饱和等问题，并能处理舵故障。

基于上述考虑，本文提出了一种在速度传感器失效和不确定情况下的 X-舵 AUV 最佳鲁棒轨迹跟踪控制器。主要贡献概述如下：

(1) 提出了两个简化阶扩展状态观测器（ESO）来估算涌浪和波浪速度，并在控制器设计中用估算值替代所有与速度相关的参数。因此，即使速度传感器出现故障，也能完成轨迹跟踪任务。与传统的轨迹跟踪控制器相比，所提出的方法可以在不使用测量线性速度的情况下实现轨迹跟踪任务。
(2) 为动力学控制设计了最优鲁棒扰动抑制控制律，通过扰动观测器解决了未知复合扰动问题，采用改进的终端滑模控制提高了系统鲁棒性和渐近收敛性，并通过多目标优化方法实现了舵分配。与传统的扰动观测器方法相比，所提出的方法利用的是估计速度而不是测量值。此外，所提出的多目标优化方法不仅节能，而且对舵故障具有鲁棒性。



#### 问题描述

##### X-rudder AUV 模型

本文将研究一种具有特殊控制面布局的 X 型舵 AUV，如图 1 所示。与传统的交叉舵 AUV 相比，X-舵 AUV 具有以下优点：

1. 更高的安全性：X-舵可以布置在潜水器的基线内，从而避免方向舵与水下障碍物发生意外碰撞；
2. 更好的机动性： X 型舵 AUV 在水平和垂直面上都具有更好的流体力学性能和稳定性，且可以控制滚动运动；
3. 舵效率更高： X 型舵采用对角布置以获得最大延伸长度，有助于实现大长宽比和高舵效； 
4. 抗沉能力更强： 由于独立控制特性和对角布置，X舵 AUV 具有更好的抗卡舵能力，也减少了卡舵造成的严重损害；
5. 噪音更低： X舵的布局有助于减少舵与螺旋桨之间的干扰，从而降低噪音。但是，X舵 AUV 的控制比十字舵 AUV 复杂，因为 X 舵的控制面方向与 AUV 的运动方向不一致，增加了交叉耦合，不能直观地控制 AUV。通常先计算所需方向的舵力和扭矩，然后求解 SIMO 舵角分布问题，从而得到舵角。为解决上述问题，本节将研究 X-舵动力学模型。

按照标准做法，AUV 在垂直面上的运动学和动力学模型可以用涌浪、倾斜和俯仰自由度的运动分量来描述。