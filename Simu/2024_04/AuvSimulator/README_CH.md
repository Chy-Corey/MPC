# Python 车辆/水下航行器模拟器

要学习使用这个仿真平台，可以从 `main.py` 入手。该平台有两个重要的库：
- lib：程序运行的核心功能库，包括了仿真循环（`mainLoop.py`），参考轨迹生成（`guidance.py`），在导航、控制中的通用算法（`gnc.py`），控制算法（`control.py`） 。
- vehicles：设备的模型，目前只关心 `remus100.py` 。

## lib
### mainLoop.py
该模块中最重要的函数是 `simulate(N, sampleTime, vehicle)` ，它接收三个参数：
- N：采样数量
- sample Time：采样时间/步长。
- vehicle：设备的实例化对象（vehicle 的 class 定义在包 vehicles 中）。

前两个比较好理解，第三个 `vehicle` 表示设备的实例对象，该对象包含了设备的所有信息：

- 设备的动力学模型
- 设备的控制输入、状态输出
- 设备的参考指令
- 设备的控制算法

以上列举了较重要的部分。

先不细想 `vehicle` 这个参数，继续往下看。

```python
def simulate(n, sample_time, vehicle):
    DOF = 6  # 六个自由度：x，y，z，phi(p, roll)，theta(q, pitch)，psi(r, yaw)
    t = 0  # 初始化时间

    # 初始化状态向量
    eta = np.array([0, 0, 0, 0, 0, 0], float)  # x，y，z，phi，theta，psi
    nu = vehicle.nu  # 速度，eta 的导数
    u_actual = vehicle.u_actual  # 控制输入初始化，阶数由 vehicle 决定

    # 初始化仿真数据维数，自由度 × 2 表示状态和状态的导数，控制输入 × 2 表示控制输入和执行器输出
    simData = np.empty([0, 2 * DOF + 2 * vehicle.dimU], float)

    # 循环，进行仿真，请注意仿真时的所有有关于 vehicle 的数据都是 vehicle 对象生成的，且保存于对象内
    for i in range(0, n + 1):

        t = i * sample_time  # simulation time

        # Vehicle specific control systems
        if vehicle.controlMode == 'depthAutopilot':
            u_control = vehicle.depthAutopilot(eta, nu, sample_time)
        elif vehicle.controlMode == 'headingAutopilot':
            u_control = vehicle.headingAutopilot(eta, nu, sample_time)
        elif vehicle.controlMode == 'depthHeadingAutopilot':
            u_control = vehicle.depthHeadingAutopilot(eta, nu, sample_time)
        elif vehicle.controlMode == 'DPcontrol':
            u_control = vehicle.DPcontrol(eta, nu, sample_time)
        elif vehicle.controlMode == 'stepInput':
            u_control = vehicle.stepInput(t)

            # Store simulation data in simData
        signals = np.append(np.append(np.append(eta, nu), u_control), u_actual)
        simData = np.vstack([simData, signals])

        # Propagate vehicle and attitude dynamics
        [nu, u_actual] = vehicle.dynamics(eta, nu, u_actual, u_control, sample_time)
        eta = attitudeEuler(eta, nu, sample_time)

    # Store simulation time vector
    simTime = np.arange(start=0, stop=t + sample_time, step=sample_time)[:, None]

    return simTime, simData
```

了解了 `mainLoop.py` 后，并不知道其中调用的 `vehicle` 对象是如何实现的，接下来就需要读懂 `vehicles` 包下的 `remus100.py` 模块。



## vehicles

### remus100.py

该对象初始化接收 6 个参数，分别为：

- controlSystem：控制算法
- r_z：参考深度
- r_psi：参考方向角
- r_rpm：参考螺旋桨转速
- V_current：洋流速度
- beta_current：洋流方向

#### dynamics

动力学模型，传入五个参数：

- eta：系统状态向量（x，y，z，phi，theta，phi）
- nu：系统状态的导数
- u_actual：执行器状态
- u_control：控制输入
- sampleTime：采样时间

```python
def dynamics(self, eta, nu, u_actual, u_control, sampleTime):
    """
    [nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime) integrates
    the AUV equations of motion using Euler's method.
    """

    # Current velocities
    u_c = self.V_c * math.cos(self.beta_c - eta[5])  # 洋流速度（纵荡）
    v_c = self.V_c * math.sin(self.beta_c - eta[5])  # 洋流速度（横）

    nu_c = np.array([u_c, v_c, 0, 0, 0, 0], float)  # 洋流速度向量
    Dnu_c = np.array([nu[5] * v_c, -nu[5] * u_c, 0, 0, 0, 0], float)  # 洋流速度向量的微分
    nu_r = nu - nu_c  # AUV 
    alpha = math.atan2(nu_r[2], nu_r[0])  # 攻角
    U = math.sqrt(nu[0] ** 2 + nu[1] ** 2 + nu[2] ** 2)  # 合速度
    U_r = math.sqrt(nu_r[0] ** 2 + nu_r[1] ** 2 + nu_r[2] ** 2)  # 相对合速度

    # Commands and actual control signals
    delta_r_c = u_control[0]  # 尾舵控制输入 (rad)
    delta_s_c = u_control[1]  # 安定面控制输入 (rad)
    n_c = u_control[2]  # 螺旋桨控制输入 (rpm)

    delta_r = u_actual[0]  # 当前时刻尾舵状态 (rad)
    delta_s = u_actual[1]  # 当前时刻安定面状态 (rad)
    n = u_actual[2]  # 当前时刻螺旋桨转速 (rpm)

    # Propeller coeffs. KT and KQ are computed as a function of advance no.
    # Ja = Va/(n*D_prop) where Va = (1-w)*U = 0.944 * U; Allen et al. (2000)
    D_prop = 0.14  # propeller diameter corresponding to 5.5 inches
    t_prop = 0.1  # thrust deduction number
    n_rps = n / 60  # propeller revolution (rps) 
    Va = 0.944 * U  # advance speed (m/s)

    # Ja_max = 0.944 * 2.5 / (0.14 * 1525/60) = 0.6632
    Ja_max = 0.6632

    # Single-screw propeller with 3 blades and blade-area ratio = 0.718.
    # Coffes. are computed using the Matlab MSS toolbox:     
    # >> [KT_0, KQ_0] = wageningen(0,1,0.718,3)
    KT_0 = 0.4566
    KQ_0 = 0.0700
    # >> [KT_max, KQ_max] = wageningen(0.6632,1,0.718,3) 
    KT_max = 0.1798
    KQ_max = 0.0312

    # Propeller thrust and propeller-induced roll moment
    # Linear approximations for positive Ja values
    # KT ~= KT_0 + (KT_max-KT_0)/Ja_max * Ja   
    # KQ ~= KQ_0 + (KQ_max-KQ_0)/Ja_max * Ja  

    if n_rps > 0:  # forward thrust

        X_prop = self.rho * pow(D_prop, 4) * (
                KT_0 * abs(n_rps) * n_rps + (KT_max - KT_0) / Ja_max *
                (Va / D_prop) * abs(n_rps))
        K_prop = self.rho * pow(D_prop, 5) * (
                KQ_0 * abs(n_rps) * n_rps + (KQ_max - KQ_0) / Ja_max *
                (Va / D_prop) * abs(n_rps))

    else:  # reverse thrust (braking)

        X_prop = self.rho * pow(D_prop, 4) * KT_0 * abs(n_rps) * n_rps
        K_prop = self.rho * pow(D_prop, 5) * KQ_0 * abs(n_rps) * n_rps

        # Rigi-body/added mass Coriolis/centripetal matrices expressed in the CO
    CRB = m2c(self.MRB, nu_r)
    CA = m2c(self.MA, nu_r)

    # CA-terms in roll, pitch and yaw can destabilize the model if quadratic
    # rotational damping is missing. These terms are assumed to be zero
    CA[4][0] = 0  # Quadratic velocity terms due to pitching
    CA[0][4] = 0
    CA[4][2] = 0
    CA[2][4] = 0
    CA[5][0] = 0  # Munk moment in yaw
    CA[0][5] = 0
    CA[5][1] = 0
    CA[1][5] = 0

    C = CRB + CA

    # Dissipative forces and moments
    D = np.diag([
        self.M[0][0] / self.T_surge,
        self.M[1][1] / self.T_sway,
        self.M[2][2] / self.T_heave,
        self.M[3][3] * 2 * self.zeta_roll * self.w_roll,
        self.M[4][4] * 2 * self.zeta_pitch * self.w_pitch,
        self.M[5][5] / self.T_yaw
    ])

    # Linear surge and sway damping
    D[0][0] = D[0][0] * math.exp(-3 * U_r)  # vanish at high speed where quadratic
    D[1][1] = D[1][1] * math.exp(-3 * U_r)  # drag and lift forces dominates

    tau_liftdrag = forceLiftDrag(self.diam, self.S, self.CD_0, alpha, U_r)
    tau_crossflow = crossFlowDrag(self.L, self.diam, self.diam, nu_r)

    # Restoring forces and moments
    g = gvect(self.W, self.B, eta[4], eta[3], self.r_bg, self.r_bb)

    # Horizontal- and vertical-plane relative speed
    U_rh = math.sqrt(nu_r[0] ** 2 + nu_r[1] ** 2)
    U_rv = math.sqrt(nu_r[0] ** 2 + nu_r[2] ** 2)

    # Rudder and stern-plane drag
    X_r = -0.5 * self.rho * U_rh ** 2 * self.A_r * self.CL_delta_r * delta_r ** 2
    X_s = -0.5 * self.rho * U_rv ** 2 * self.A_s * self.CL_delta_s * delta_s ** 2

    # Rudder sway force 
    Y_r = -0.5 * self.rho * U_rh ** 2 * self.A_r * self.CL_delta_r * delta_r

    # Stern-plane heave force
    Z_s = -0.5 * self.rho * U_rv ** 2 * self.A_s * self.CL_delta_s * delta_s

    # Generalized force vector
    tau = np.array([
        (1 - t_prop) * X_prop + X_r + X_s,
        Y_r,
        Z_s,
        K_prop / 10,  # scaled down by a factor of 10 to match exp. results
        self.x_s * Z_s,
        self.x_r * Y_r
    ], float)

    # AUV dynamics
    tau_sum = tau + tau_liftdrag + tau_crossflow - np.matmul(C + D, nu_r) - g
    nu_dot = Dnu_c + np.matmul(self.Minv, tau_sum)

    # Actuator dynamics
    delta_r_dot = (delta_r_c - delta_r) / self.T_delta
    delta_s_dot = (delta_s_c - delta_s) / self.T_delta
    n_dot = (n_c - n) / self.T_n

    # Forward Euler integration [k+1]
    nu += sampleTime * nu_dot
    delta_r += sampleTime * delta_r_dot
    delta_s += sampleTime * delta_s_dot
    n += sampleTime * n_dot
    
	# Amplitude saturation of the control signals
    if abs(delta_r) >= self.deltaMax_r:
        delta_r = np.sign(delta_r) * self.deltaMax_r

    if abs(delta_s) >= self.deltaMax_s:
        delta_s = np.sign(delta_s) * self.deltaMax_s

    if abs(n) >= self.nMax:
        n = np.sign(n) * self.nMax
    u_actual = np.array([delta_r, delta_s, n], float)

    return nu, u_actual
```

<img src="E:\Library\硕士实验室\MPC\Simu\2024_04\AuvSimulator\image\01.png" alt="01" style="zoom:90%;" />















#### depthHeadingAutopilot

深度、方向控制算法，接收三个参数：

- eta：系统状态
- nu：系统状态的导数
- sampleTime：采样时间

实际上还会从对象中调取参考指令：参考方向角，参考深度。

