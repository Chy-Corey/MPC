### AUV 控制框架



参考自动驾驶：

<img src="E:\Library\硕士实验室\MPC\Note\2024_2\image\03-01.png" alt="03-01" style="zoom:67%;" />

总共分 3 步：

1. 最外层：根据任务与环境信息生成参考路径
2. 第二层：根据参考路径、当前位置、当前状态以及约束生成参考轨迹
3. 第三层：根据参考轨迹对设备进行控制。



在《AUVTrajectory Tracking Models and Control Strategies》中，AUV 控制框架如下所示：

<img src="E:\Library\硕士实验室\MPC\Note\2024_2\image\03-02.png" alt="03-02" style="zoom:67%;" />

与车辆的自动驾驶类似，多了一个图形化界面（GUI），并且路径规划和轨迹规划是在远端计算机在线完成的，需要建立网络通信。



### REMUS100 参数（传感器）

[THE REMUS 100 - Oceantech](http://www.oceantech.global/the-remus-100/)

市面上的 AUV 直接售卖封装好的传感器，可以选装到 AUV 上。

<img src="E:\Library\硕士实验室\MPC\Note\2024_2\image\03-03.png" alt="03-03" style="zoom:80%;" />

相控阵多普勒速度测数（DVL）：速度、洋流速度、AUV距离海面和海底的位置、深度、避障功能、地形跟踪功能、行进路线的航位推算



<img src="E:\Library\硕士实验室\MPC\Note\2024_2\image\03-04.png" alt="03-04" style="zoom:80%;" />

用于测量级导航的惯性导航、iXblue内置静音真固态光纤陀螺仪技术、内置 Kongsberg Maritime NavP 辅助惯性导航系统、精确、可靠的声学定位

<img src="E:\Library\硕士实验室\MPC\Note\2024_2\image\03-05.png" alt="03-05" style="zoom:80%;" />

光学传感器，用于探测水质等等。

还可以选装更多功能的传感器。

总之算法设计需要用到的变量，都可以使用传感器测算得到。

