# My Implementation Of Gmapping - Paper Reading

## What's the problem? 

Particle filter is a Monte Carlo method that estimates the probability distribution of a random variable. If you are curious what that looks like, please check out my previous article.

 Here, the random variable is jointly **robot pose trajectory** and **map history** $(x_{1:t},m_{1:t})$. At any time instant, we know the previous pose $x_{1:t-1}$, control inputs $u_{1:t}$, and observations $z_{1:t}$. So, the problem is to estimate $p(x_{1:t}, m_{1:t} | z_{1:t}, u_{1:t})$. Sampling this joint distribution is almost impractical! So, to simplify, We can apply Rao-Blackwellization to this context, which is basically "The estimator of the map $m_{1:t}$, given sufficient statistic in robot trajectory $x_{1:t}$, is no worse than the original estimator of the map and path jointly".

Hence, we have the following factorization:

$$p(x_{1:t}, m_{1:t} | z_{1:t}, u_{1:t}) = p(m_{1:t}|x_{1:t}, z_{1:t})p(x_{1:t}|z_{1:t}, u_{1:t})$$

This allows us to first evaluate the robot trajectory evaluate $p(x_{1:t}|z_{1:t}, u_{1:t})$, then update $p(m_{1:t}|x_{1:t}, z_{1:t})$. And since we know the trajectory already, updating the map will be more accurate.

Pose trajectory is estimated using particle filter. Since we think $z_t$ is independent of $z_{1:t-1}$ and $u_{1:t-1}$, we have $p(z_t|z_{1:t-1}, u_{1:t}) = p(z_t)$, which is constant. We can denote it as $\eta$

$$
p(x_{1:t}|z_{1:t}, u_{1:t}) = p(x_{1:t}|z_t, z_{1:t-1}, u_{1:t})
\\
= \frac{p(z_t|x_{1:t}, z_{1:t-1}, u_{1:t})p(x_{1:t}|z_{1:t-1}, u_{1:t})}{p(z_t|z_{1:t-1}, u_{1:t})}
\\
= \eta p(z_t|x_{1:t}, z_{1:t-1}, u_{1:t})p(x_{1:t}|z_{1:t-1}, u_{1:t})
$$

Also, **we think $z_t$ is only dependent on $x_t$ and $x_t$ is only dependent on $x_{t-1}$ and $u_{1:t}$**. So, this can be further leaned down to:
$$
p(z_t|x_{1:t}, z_{1:t-1}, u_{1:t}) = p(z_t|x_t)
\\
p(x_{1:t}|z_{1:t-1}, u_{1:t}) = p(x_t|x_{1:t-1}, z_{1:t-1}, u_{1:t})p(x_{1:t-1}|z_{1:t-1}, u_{1:t}) = p(x_t|x_{1:t}, u_t)p(x_{1:t-1}|z_{1:t-1}, u_{1:t-1})
\\
=>
\\
\eta p(z_t|x_{1:t}, z_{1:t-1}, u_{1:t})p(x_{1:t}|z_{1:t-1}, u_{1:t}) = \eta p(z_t|x_t)p(x_t|x_{1:t}, u_t)p(x_{1:t-1}|z_{1:t-1}, u_{1:t-1})
$$

Note: $p(x_{1:t-1}|z_{1:t-1}, u_{1:t-1})$ is the posterior from the last update; $p(z_t|x_t)$ is the sensor probability model, and $p(x_t|x_{1:t}, u_t)$ is the robot's dynamic probablity model.

## SIR; why the weights? Restraints

- Importance sampling
    - Ideally, draw from your post trajectory distribution. But that's what you are looking for. It's easier to draw samples from $\pi$. So intuitively, if we draw a sample from $\pi$, and know (its apperance in target_distribution)/(its appearance in the current distribution), we can get an idea how likely it will be drawn in the target probablity. So we use this ratio as the "importance weight"


- Proposal Distribution Design: the worse, the better.

Resample Adaptively To reduce Particle Depletion 
(neff)

Final annotated algo

May not need to compute miu and covariance
 
References