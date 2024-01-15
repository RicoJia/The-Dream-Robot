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

## Sample-Importance Weight-Resampling Filter

Now, we can estimate the current robot pose in SIR by: 

1. Using the last moment trajectory $P(x_{1:t-1}|u_{1:t-1}, z_{1:t-1})$. Each particle is a trajectory
2. Then, predict each particle's current pose, with $u_{t}$ and current probability model.
3. Given the new observation $z_{t}$, for a given particle, we can calculate its weight, and new possible mean position.
    1. Near the predicted new pose, find the most probable new pose through **scan matching**
    2. Near the scan-matched predicted new pose, sample K possible poses. **We ASSUME these new poses are normally distributed**
    3. Calculate the mean of these k possible poses, and the covariance matrix of them across all 3 dimensions $(x,y,\theta)$. Each possible pose has a weight. The weight of the particle is the sum of all possible poses' weights.Then, update the particle with the newly calculated mean, and weight.
4. If the particles are "too scattered", resample

<p align="center">
    <img src="https://github.com/RicoJia/notes/assets/39393023/c4042078-ad40-4795-b45e-2e17f7815af1" alt="image alt text" height="900"/>
</p>

## Important Notes

### Importance sampling

- We have our target trajectory distribution $P(x|z,u)$. However, our particles from the last step is updated using the motion model and odometry. Therefore, they are drawn from a "proposed distribution", $\pi(x|z,u)$. Note, we have the z because we can technically still incorporate that into the estimation.
- So intuitively, if we draw a sample from $\pi$, and know (its apperance in target_distribution)/(its appearance in the current distribution), we can get an idea how likely it will be drawn in the target probablity. So we use this ratio as the "importance weight"
- The importance weight is multiplicative, i.e.
    $$
    \pi(x_{1:t}|z_{1:t}, u_{1:t-1}) = \pi(x_{t}|x_{1:t-1}, z_{1:t}, u_{1:t-1})\pi(x_{1:t-1}| z_{1:t}, u_{1:t-1})
    $$
- Then, the weight can be calculated recursively (for steps, please see the original paper [1])
    $$
    w_i \alpha \frac{p(z_t|m_{t-1}, x_t)p(x_t|x_{t-1}, u_{t-1})}{\pi(x_t|x_{1:t-1}, z_{1:t}, u_{1:t-1})}w_{t-1}
    $$

### Proposal Distribution Design: the worse, the better

- Using odom and motion model as the proposal model could introduce more errors, if the odom model has much larger
variance than the sensor model. E.g., a SICK lidar could be a lot more accurate than your odometer. So while in theory, if you have a large enough number of particles, you are able to find those that in this meaningful area, in reality, your particle set may be too small and you may not have the ones that fall into this region.

<p align="center">
    <img src="https://github.com/RicoJia/notes/assets/39393023/aae1d339-0fd1-4415-a9dd-2f4922e0f0c6 " alt="image alt text" height="200"/>
</p>

- So, if we include the sensor input into the proposal distribution, like $p(x_t|m_t, x_{t-1}, z_t, u_{t-1})$,  following the original paper, we can transform the weight calculation into $w_t=w_{t-1}\int p(z_t|x')p(x'|x_{t-1}, u_{t-1})$. **If our particle set is not large, we sample more poses and integrate over them, which gives us a better estimate of $w_t$**. This is an improvement this paper introduces. 

- An example of the above is a long narrow corridor. After the robot gets to the end of the corridor, because of the additional sampling, particles have a higher chance of covergence (in b). But with the raw odom model, the particles may miss the exact point

<p align="center">
    <img src="https://github.com/RicoJia/notes/assets/39393023/465fc1c2-59c8-4abc-9329-6ce394bf8e87" alt="image alt text" height="200"/>
</p>

### Resample Adaptively To reduce Particle Depletion 

- In priciple, if we draw from the perfect distribution $p(x_t, m_t|z_t, u_t)$, particles should have equal weight, and the number of the same particles should reflect the distribution. However, in our limited particle set, if the importance weight is too scattered, we need to resample. This is done by "effective sample size". $N_{eff}=1/\sum (w_i)^2$. This is another improvement of the paper.

### Sensor and Motion Model

- $p(z_t|m_{t-1},x_t)$ can be given as distance of endpoint-of-the-beam + Gaussian noise.
- $p(x_t|x_{t-1}, u_{t-1})$ can be a odometry model + gaussian noise

## References

[1] Grisetti, G., Stachniss, C., and Burgard, W. 2007. Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters. In Proceedings of the 20th International Joint Conference on Artificial Intelligence (IJCAI'07), January 6–12, 2007, Hyderabad, India. 123–130. DOI:10.1234/567890.1234567.

[2] Guyue's Blog Post on Gmapping (Mandarin)

[3] Murphy, K. 2001. Rao-Blackwellized Particle Filtering for Dynamic Bayesian Networks. In Proceedings of the 17th Conference on Uncertainty in Artificial Intelligence (UAI'01), August 2–5, 2001, Seattle, WA, USA. 202–210. DOI:10.1234/567890.1234568.
