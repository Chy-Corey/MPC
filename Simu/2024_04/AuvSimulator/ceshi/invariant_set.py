import numpy as np
import cvxpy as cp

# 定义系统矩阵
A = np.array([[0.5, 0.1],
              [0.1, 0.4]])

# 定义Q矩阵
Q = np.eye(2)

# 定义Lyapunov变量
P = cp.Variable((2, 2), symmetric=True)

# 定义LMI约束
constraints = [P >> 0, A.T @ P @ A - P << -Q]

# 定义优化问题
prob = cp.Problem(cp.Minimize(0), constraints)

# 求解优化问题
prob.solve()

# 检查是否求解成功
if prob.status == cp.OPTIMAL:
    P_value = P.value
    print("Lyapunov矩阵P:")
    print(P_value)
else:
    print("未找到合适的P")

# 定义状态约束
b1 = 1
b2 = 1

# 计算状态约束边界上的Lyapunov函数值
V1 = [P_value[0, 0]*b1**2 + 2*P_value[0, 1]*b1*b2 + P_value[1, 1]*b2**2,
      P_value[0, 0]*(-b1)**2 + 2*P_value[0, 1]*(-b1)*b2 + P_value[1, 1]*b2**2,
      P_value[0, 0]*b1**2 + 2*P_value[0, 1]*b1*(-b2) + P_value[1, 1]*(-b2)**2,
      P_value[0, 0]*(-b1)**2 + 2*P_value[0, 1]*(-b1)*(-b2) + P_value[1, 1]*(-b2)**2]

c_init = min(V1)

# 定义Lyapunov函数
def V(x):
    return x.T @ P_value @ x

# 迭代调整 c 并验证不变性
c = c_init
is_invariant = False

while c > 0:
    is_invariant = True
    for x1 in np.arange(-b1, b1+0.1, 0.1):
        for x2 in np.arange(-b2, b2+0.1, 0.1):
            x = np.array([x1, x2])
            if V(x) <= c:
                if V(A @ x) > c:
                    is_invariant = False
                    break
        if not is_invariant:
            break
    if is_invariant:
        break
    else:
        c *= 0.9  # 减小 c

if is_invariant:
    print(f'找到正不变集，c = {c}')
else:
    print('未找到正不变集')

print('矩阵 P 为：')
print(P_value)
