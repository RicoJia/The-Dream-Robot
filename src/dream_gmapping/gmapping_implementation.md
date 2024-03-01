# GMapping Implementation

## Iterative Closest Point (ICP) 
The goal of ICP is: given two point clouds, we can find a transform T that aligns the two together. Let $T=RX+[T_x, T_y]$, where $X$ is a point in the source cloud.We want to minimize the total distance between the two clouds after the transform: $e = \sum_X (R(\theta)X_n+[T_x, T_y]-Y_n)^2$. ICP's job is to find the set of parameters $[T_x,T_y,\theta]$ to minimize this error


![icp](https://github.com/RicoJia/The-Dream-Robot/assets/39393023/095634fa-3a1f-4bc6-9df6-b05835da7669)

[Image Source](https://www.researchgate.net/figure/An-advantage-point-to-plane-over-point-to-point-method_fig1_267784785)

Point to point mapping is the most basic form of ICP: we assume a point in one cloud is associated with the closest point in the other cloud. One way is by the Least Squares method (LS). 

1. First Linearize the error function around a given set of parameters, $p=[T_x, T_y, \theta]$: 

    1. Transform $e = \sum_X (R(\theta)X_n+[T_x, T_y]-Y_n)^2$ is 

    $$
    e = 
    \begin{pmatrix}
    T_x + cos(\theta)-sin(\theta) \\
    T_y + sin(\theta)+cos(\theta) \\
    \end{pmatrix}
    $$

    2. So the Jacobian of $e$ is
    $$
    J(p) = [\frac{\partial e}{\partial T_x}, \frac{\partial e}{\partial T_y}, \frac{\partial e}{\partial \theta}]
    =
    \begin{pmatrix}
    1 & 0 & -sin(\theta)-cos(\theta) \\
    0 & 1 & cos(\theta)-sin(\theta) \\
    \end{pmatrix}
    $$

2. Write out the first order Taylor series of error $e$, w.r.t parameters, $p=[T_x, T_y, \theta]$:
    $$
    e \approx e(p_0) + J(p)^T(p - p_0)
    $$

3. So to get the minimum $|e|$, it's equivalent to getting $\delta p = p-p_0 = argmin[(e(p_0) + J^T(p - p_0))^2]$. Taking the first order derivative gives: $J(p)e(p) + J(p)^TJ(p) \delta p = 0$. Doing this over all n cloud points: $H = \sum_N(J_n(p)^TJ_n(p))$, $b=\sum_N J_n(p)e_n(p)$, and $\delta p = -H^{-1}b$

4. Keep repeating step 3 until convergence

### Potential Bug List
- we are returning if a pixel is full by a set standard. (if the pixel is unknown, it is not full). TODO: check if we need to change the unknown stage