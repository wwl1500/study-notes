# 第二章 齐次变换矩阵（Homogeneous Transformation Matrices）

> 本章是机器人几何与运动学建模的核心内容。通过齐次变换矩阵（Homogeneous Transformation Matrix），可以统一描述物体的**平移**与**旋转**，并为后续的机器人运动学、动力学和控制提供基础。

---

## 2.1 平移变换（Translational Transformation）

平移是物体沿某个方向的线性位移。设三维空间中的点：

\[
P(x, y, z)
\]

平移向量表示为：

\[
\mathbf{d} = a\mathbf{i} + b\mathbf{j} + c\mathbf{k}
\]

其中 \(a, b, c\) 分别表示沿 \(x, y, z\) 方向的位移。

将三维点扩展为齐次坐标形式（4×1 向量）：

\[
\mathbf{q} = 
\begin{bmatrix}
x \\ y \\ z \\ 1
\end{bmatrix}
\]

则平移矩阵可表示为：

\[
\mathbf{H} = 
\begin{bmatrix}
1 & 0 & 0 & a \\
0 & 1 & 0 & b \\
0 & 0 & 1 & c \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

平移后的点为：

\[
\mathbf{v} = \mathbf{Hq} = 
\begin{bmatrix}
x + a \\
y + b \\
z + c \\
1
\end{bmatrix}
\]

### 🧩 示例

给定：
\[
\mathbf{q} = 
\begin{bmatrix} 2 \\ 3 \\ 2 \\ 1 \end{bmatrix},
\quad
\mathbf{d} = (4, -3, 7)
\]

平移矩阵：
\[
\mathbf{H} =
\begin{bmatrix}
1 & 0 & 0 & 4 \\
0 & 1 & 0 & -3 \\
0 & 0 & 1 & 7 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

则平移结果：
\[
\mathbf{v} = \mathbf{Hq} = 
\begin{bmatrix}
6 \\ 0 \\ 9 \\ 1
\end{bmatrix}
\]

---

## 2.2 旋转变换（Rotational Transformation）

旋转是物体绕某一固定轴的运动。在三维空间中，绕三个坐标轴的旋转矩阵分别为：

### 绕 x 轴旋转角度 α：

\[
R_x(\alpha) =
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\alpha & -\sin\alpha \\
0 & \sin\alpha & \cos\alpha
\end{bmatrix}
\]

### 绕 y 轴旋转角度 β：

\[
R_y(\beta) =
\begin{bmatrix}
\cos\beta & 0 & \sin\beta \\
0 & 1 & 0 \\
-\sin\beta & 0 & \cos\beta
\end{bmatrix}
\]

### 绕 z 轴旋转角度 γ：

\[
R_z(\gamma) =
\begin{bmatrix}
\cos\gamma & -\sin\gamma & 0 \\
\sin\gamma & \cos\gamma & 0 \\
0 & 0 & 1
\end{bmatrix}
\]

> 🔹 **右手定则：**
> 右手拇指指向旋转轴的正方向，四指弯曲的方向即为正旋转方向。

齐次旋转矩阵扩展为 4×4 形式：

\[
H_R =
\begin{bmatrix}
R_{3x3} & 0 \\
0 & 1
\end{bmatrix}
\]

---

### 🧩 示例：绕 z 轴旋转

向量：
\[
\mathbf{u} = 
\begin{bmatrix}
7 \\ 3 \\ 0
\end{bmatrix}
\]

绕 z 轴旋转 \(90^\circ\)：
\[
R_z(90^\circ) = 
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 1
\end{bmatrix}
\]

计算结果：
\[
\mathbf{w} = R_z(90^\circ)\mathbf{u} =
\begin{bmatrix}
-3 \\ 7 \\ 0
\end{bmatrix}
\]

---

## 2.3 位姿与位移（Pose and Displacement）

**位姿（Pose）** = **位置（Position）** + **方向（Orientation）**

可以用一个齐次变换矩阵表示：

\[
\mathbf{H} =
\begin{bmatrix}
R_{3x3} & \mathbf{p} \\
0 & 1
\end{bmatrix}
\]

其中：
- \( R_{3x3} \)：描述物体的方向（旋转矩阵）  
- \( \mathbf{p} = [x, y, z]^T \)：描述物体的位置（平移向量）

#### ✅ 示例：

\[
\mathbf{H} =
\begin{bmatrix}
0 & 0 & 1 & 4 \\
1 & 0 & 0 & -3 \\
0 & 1 & 0 & 7 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

解释：
- 旋转部分（前三列）表示新坐标系的方向；
- 第四列 `[4, -3, 7]^T` 表示新坐标系原点相对于参考系的位置。

---

## 2.4 几何机器人模型（Geometrical Robot Model）

齐次变换矩阵可用于描述机器人各连杆、关节之间的几何关系。

设机器人有若干个关节和连杆：

\[
{}^{0}H_n = ({}^{0}H_1 D_1) \cdot ({}^{1}H_2 D_2) \cdot ({}^{2}H_3 D_3) \cdots ({}^{n-1}H_n D_n)
\]

每个 \( {}^{i-1}H_i \) 表示第 \(i\) 个关节的**位置与方向**，  
每个 \( D_i \) 表示第 \(i\) 个关节的**旋转或平移变换**。

---

### 🦾 SCARA 机器人几何模型

SCARA（Selective Compliant Articulated Robot Arm）是一种典型的四关节工业机器人。

其几何模型的齐次矩阵为：

\[
{}^{0}H_{3}=
\begin{bmatrix}
c_{12} & -s_{12} & 0 & -l_3s_{12}-l_2s_1 \\
s_{12} & c_{12} & 0 & l_3c_{12}+l_2c_1 \\
0 & 0 & 1 & l_1-d_3 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

其中：

\[
c_{12} = \cos(\theta_1 + \theta_2), \quad s_{12} = \sin(\theta_1 + \theta_2)
\]

参数含义：

| 符号                     | 含义                 |
| ------------------------ | -------------------- |
| \( l_1, l_2, l_3 \)      | 各连杆长度           |
| \( d_3 \)                | 第三关节的平移量     |
| \( \theta_1, \theta_2 \) | 前两个关节的旋转角度 |

---

## ✅ 总结

- 齐次变换矩阵将平移与旋转统一在一个矩阵框架中；
- 通过矩阵连乘可以描述机器人各关节和末端执行器的位姿；
- 是机器人学中分析 **正运动学、逆运动学、动力学与控制** 的基础工具。

---
> 📘 **关键要点回顾**
>
> - 平移与旋转都可用 4×4 矩阵表示；
> - 齐次矩阵形式：  
>   \[
>   H =
>   \begin{bmatrix}
>   R & p \\
>   0 & 1
>   \end{bmatrix}
>   \]
> - 组合变换通过矩阵相乘实现；
> - 齐次矩阵是描述机器人几何结构的标准形式。

---