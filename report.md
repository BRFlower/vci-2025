## Task 1:逆动力学

### sub-task 1 前向更新

计算每个关节在地面参照系下的旋转和平移（即机械臂矢量）

对于第i个关节相对于第i-1个关节（平行于地面参照系），平移向量为r_{ii-1}旋转i-1关节的地面系旋转GlobalRotation\[i-1\];旋转向量为GlobalRotation\[i-1\]叠加i关节的旋转，在四元数的计算体系中可直接相乘，满足结合律

### sub-task 2 CCD IK算法

思路：多次迭代，每次从末端往前，依次旋转当前关节i，使得末端离目标点最接近，实际上就是旋转向量GlobalOffset\[-1\] - GlobalOffset\[i\]与目标向量EndPosition - GlobalOffset\[i\]平行，通过glm::rotation(vec1,vec2)直接计算旋转减小误差

理论上每次更新i关节的旋转，都要重新计算前向动力学 ForwardKinematics，比较浪费，实际上每一次迭代的循环中只会用到JointRotation\[-1\]，可用```JointGlobalPosition[ik.JointLocalOffset.size()-1] = ik.JointGlobalPosition[i] + rt0 * r * rt0_ * (ik.JointGlobalPosition[ik.JointLocalOffset.size()-1] - ik.JointGlobalPosition[i]);```直接更新末端，等到循环结束一并更新所有的Offset和Rotation

### sub-tank 3 FABR IK 算法

FABRIK算法应用了一种“牵拉”的思想，每次迭代进行后向和前向两次更新：

第一步后向，把终点放在目标点，然后为了保持机械臂长度约束，将前一个点往目标点移动使得机械臂长度不变，以此类推；最终起始点会离原来的起始点一定距离

第二步前向，把起点放在正确的位置修复约束，然后为了保持机械臂长度约束，将后一个点往目标点移动使得机械臂长度不变，以此类推，最后机械臂末端离目标点会有一个更小的误差。

多次迭代以满足目标精度。对于连续运动学约束，初始值采用上一帧的位置，很容易在短时间内得到很高精度的结果，而且关节旋转少，稳定性很好

### sub-task 4 自定义曲线

重写BuildCustomTargetPosition(),创建直线参数方程L(t, p1,p2)与椭圆参数方程(t, o, r1, r2, theta1,theta2, scale)，在坐标系中分段绘画字母B
![[img1.png]]


4.1 均匀化

对于性质一般的参数曲线，可能有些地方移动速率$\frac{d\vec{r}}{dt}$大有些地方小，如果t均匀取样，则有些地方过度密集，有些地方过度稀疏
考虑先进行小规模的取样，差分估计点的移动速度，则相应设置步长反比于速度，使得$\delta r = \frac{\delta\vec{r}}{\delta t} * t_step$近似保持不变
考虑到画笔可能进行长距离移动导致节点处步长过小，对步长进行clamp(和平均值比较)，过大说明发生跳跃，用临近点的速度代替。

修改前：
![[img2.png]]
修改后：
![[img3.png]]

4.2

1 如果目标太远，机械臂会保持完全伸直的状态，方向指向目标点

2 CCD IK末端旋转幅度较大，通常要较多次数才能达到目标精度；FABR IK同时考虑了整个机械臂的姿态，一起旋转，每一处旋转幅度较小，求解稳定，通常迭代次数更少

3 本质上是解决时间上位姿不连续的问题，FABR IK是使用连续帧的稳定算法效果更好；进一步地，可以限制关节的旋转速度，即两帧之间角度变化不超过阈值；考虑输入的波动，或许对结果插值和滤波是有用的做法


Task2 弹簧质点系统

隐式欧拉方法：


$$
\begin{gathered}
\begin{cases}
X_{k+1} = X_{k} + hV_{k+1} \\
V_{k+1} = V_{k} + hM^{-1}f(X_{k+1})
\end{cases}\\
希望把f(X_{k+1})表示成X_{k+1}的线性组合\\
MX_{k+1} = h^{2}f(X_{k+1})+M(hV_{k}+X_{k})
为了表示成矩阵，X_{k+1}变成3*n向量，第3*i+j表示第i点的j维\\
\\
移项构造A(X_{k+1}-X_{k}) = B:\\
MX_{k+1},MX_{k}贡献：A加上质量矩阵M\\
MhV_{k}贡献：B加上MhV_{k}\\
重力贡献项：B加上向量 h^{2} * g * Mass_{i}\\
弹簧弹力项：需要用X_{k+1}的一阶线性表示\\
f(X) = - \nabla E(X) \implies f(X_{k+1}) = f(X_{k}) + \nabla f(X_{k}) (X_{k+1}-X_{k})\\
其中\nabla f(X_{k}) = −\nabla^{2}E(X_{k})=-H_{E}(X_{k})要算海森矩阵\\
~~即A加上h^{2}H_{E}，B加上h^{2}f(X_{k})
弹簧x_{i} \sim x_{j}的贡献 E = \frac{1}{2}k(||x_{i}-x_{j}|| - l_{0})^{2}平方根计算困难，\\
进行方向冻结-只保留x_{i}-x_{j}方向的距离变化，并只计算这个方向的力\\
E = \frac{1}{2} k(d^{T}(x_{i}-x_{j})-l_{0})^{2}其中d是x_{i}-x_{j}的方向向量，在求导中视为常数\\
于是E是关于X (3*n向量)的正定二次型，容易计算海森矩阵,每个弹簧对H的贡献是\begin{pmatrix}
\frac{\partial^{2}E}{\partial X_{i}\partial X_{i}} & \frac{\partial^{2}E}{\partial X_{i}\partial X_{j}} \\
\frac{\partial^{2}E}{\partial X_{j}\partial X_{i}} & \frac{\partial^{2}E}{\partial X_{j}\partial X_{j}}
\end{pmatrix}\begin{pmatrix}
dd^{T} & -dd^{T} \\
-dd^{T} & dd^{T}
\end{pmatrix}\\
综上,h^{2}f(X_{k+1})贡献是B加上h^{2}f(X_{k})，A加上h^{2}H
\end{gathered}
$$


