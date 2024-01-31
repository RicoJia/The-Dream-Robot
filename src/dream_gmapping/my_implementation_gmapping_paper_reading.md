# My Implementation Of Gmapping - Paper Reading

## What's the problem?

Particle filter is a Monte Carlo method that estimates the probability distribution of a random variable. If you are curious what that looks like, please check out my previous article.

 Here, the random variable is jointly **robot pose trajectory** and **map history** $(x_{1:t},m_{1:t})$. At any time instant, we know the previous pose $x_{1:t-1}$, control inputs $u_{1:t}$, and observations $z_{1:t}$. So, the problem is to estimate $p(x_{1:t}, m_{1:t} | z_{1:t}, u_{1:t})$. Sampling this joint distribution is almost impractical! So, to simplify, We can apply Rao-Blackwellization to this context, which is basically "The estimator of the map $m_{1:t}$, given sufficient statistic in robot trajectory $x_{1:t}$, is no worse than the original estimator of the map and path jointly".

Hence, we have the following factorization:

$$p(x_{1:t}, m_{1:t} | z_{1:t}, u_{1:t}) = p(m_{1:t}|x_{1:t}, z_{1:t})p(x_{1:t}|z_{1:t}, u_{1:t})$$

This allows us to first evaluate the robot trajectory evaluate $p(x_{1:t}|z_{1:t}, u_{1:t})$, then update $p(m_{1:t}|x_{1:t}, z_{1:t})$. And since we know the trajectory already, updating the map will be more accurate.

Pose trajectory is estimated using particle filter. Since we think $z_t$ is independent of $z_{1:t-1}$ and $u_{1:t-1}$, we have $p(z_t|z_{1:t-1}, u_{1:t}) = p(z_t)$, which is constant. We can denote it as $\eta$

```math
\begin{matrix}
p(x_{1:t}|z_{1:t}, u_{1:t}) = p(x_{1:t}|z_t, z_{1:t-1}, u_{1:t}) \\
\\
= \frac{p(z_t|x_{1:t}, z_{1:t-1}, u_{1:t})p(x_{1:t}|z_{1:t-1}, u_{1:t})}{p(z_t|z_{1:t-1}, u_{1:t})}
\\
= \eta p(z_t|x_{1:t}, z_{1:t-1}, u_{1:t})p(x_{1:t}|z_{1:t-1}, u_{1:t})
\end{matrix}
```

Also, **we think $z_t$ is only dependent on $x_t$ and $x_t$ is only dependent on $x_{t-1}$ and $u_{1:t}$**. So, this can be further leaned down to:

```math
\begin{matrix}
p(z_t|x_{1:t}, z_{1:t-1}, u_{1:t}) = p(z_t|x_t)
\\
p(x_{1:t}|z_{1:t-1}, u_{1:t}) = p(x_t|x_{1:t-1}, z_{1:t-1}, u_{1:t})p(x_{1:t-1}|z_{1:t-1}, u_{1:t}) = p(x_t|x_{1:t}, u_t)p(x_{1:t-1}|z_{1:t-1}, u_{1:t-1})
\\
=>
\\
\eta p(z_t|x_{1:t}, z_{1:t-1}, u_{1:t})p(x_{1:t}|z_{1:t-1}, u_{1:t}) = \eta p(z_t|x_t)p(x_t|x_{1:t}, u_t)p(x_{1:t-1}|z_{1:t-1}, u_{1:t-1})
\end{matrix}
```

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

## Probablistic Models

### Motion Model

Motion model $p(x_t|x_{t-1}, u_{t-1})$ can be given as a odometry model + gaussian noise

#### A Generic Motion Model [not used in implementation]

In Probablistic Robotics [5], a proposed motion model is in time interval $\Delta T$, odometry reports that:

1. Robot rotates by $\delta_1$ to face the next position
2. Robot travels along a straight line by $\delta_{trans}$ to the next position
3. Robot rotates by $\delta_{2}$ to align itself to the current orientation

Then, we add independent noises to each $\delta$, and update $x'$, $y'$ based on the previous position $x_0$, $y_0$, and $\theta_0$

```math
\begin{matrix}
x' = x_0 + \delta_{trans} cos(\theta_{0} + \delta_1) \\
y' = y_0 + \delta_{trans} sin(\theta_{0} + \delta_1) \\
\theta = \theta_0 + \delta_1 + \delta_2
\end{matrix}
```

This model is generic enough for all different kinematic models. However, for specific models, like the differential drive, this model's assumption of constant noise distribution may not hold for different $\delta_1, \delta_{trans}, \delta_2$ values. For different drive,  **instantaneous center of curvature** is a better suited model

#### Instantaneous center of curvature (ICC) [used in implementation]

We assume that in a short time interval $\Delta T$ from $t_0$ to  $t_1$, a differential drive robot goes around a circular path, as illustrated below (picture credit: [Thomas Hellström](https://www.diva-portal.org/smash/get/diva2:796235/FULLTEXT01.pdf))

<p align="center">
    <img src="https://github.com/RicoJia/The-Dream-Robot/assets/39393023/908d9f23-593a-4f28-9970-a30767e88cce" alt="image alt text" height="200"/>
</p>

The differential drive robot's angular and translational velocities are:

```math
\begin{matrix}
v = \frac{v_{left} + v_{right}}{2} \\
\dot\theta(t) = \omega = \frac{v_{left} - v_{right}}{l}
\end{matrix}
```

Then assuming these velocities are constant along the arc, we can achieve new positions $x', y', \theta'$ at $t_1$
```math
\begin{matrix}
\theta = \int_{t_0}^{t_1} \dot\theta(t) dt => \theta' = \theta_0 + \omega (t_1 - t_0)
\\
x = \int_{t_0}^{t_1} v cos(\theta(t))dt = \int_{t_0}^{t_1}\frac{vcos(\theta) \dot\theta(t)dt}{\dot\theta(t)} = \frac{v}{\omega} \int_{t_0}^{t_1} cos(\theta)d\theta => x' = \frac{v}{\omega} [sin(\theta_0 + \omega (t_1 - t_0)) - sin(\theta_0)]
\\
y = \int_{t_0}^{t_1} v sin(\theta(t))dt = \int_{t_0}^{t_1}\frac{vsin(\theta) \dot\theta(t)dt}{\dot\theta(t)} = \frac{v}{\omega} \int_{t_0}^{t_1} sin(\theta)d\theta = \frac{v}{\omega} [cos(\theta_0) - cos(\theta_0 + \omega (t_1 - t_0))]
\end{matrix}
```

How to add noise? **We can add noise to $\omega$ and $v$.** The reason is we assume $v_{left}, v_{right}$ have independent Gaussian noises. Their linear combinations, $\omega$ and $v$ are also independent Gaussian distributions.


**Important implementation consideration**: our laser scan is 10hz, while odom is 30hz. We could technically be throwing away some odom data. However, for a vanilla implementation, we just use the latest available odom data when laser scan is received. **The odom path should be compared before proceeding to the rest of RBPF so we can check how well this method performs**

### Map Updates

Map updates are done after updating particles. So, you can find the best particle for updating the map, so it's a "mapping with known poses" problem. Below we are assuming that each cell is independent of each other.

#### Counting Method (Reflection Map, or Most Likely Map)

Counting method is very interesting and relatively simple to implement. Idea is we think in a most likely map, an arbitrary cell $j$ has a probablity of "occupancy" or "beam reflection", $m_j$, and that probability maximizes the probability of the whole map. 

<p align="center">
    <img src="https://github.com/RicoJia/The-Dream-Robot/assets/39393023/b17a4d62-eab0-4b04-bd55-c97316bf17d1" alt="image alt text" height="100"/>
</p>

Therefore, we want for a given t:

$$
p(z_t | m, x_t) = \left\{
    \begin{array}{ll}
        m_y \prod_J (1-m_j) & \quad \text{if beam ends at cell y and not max range}\\
        \prod_J (1-m_j) & \quad \text{if beam has length max range } 
    \end{array}
\right.
$$

So over time $T$, we want to find $m_j$ for each cell to get the maximum likelihood map: $p(m|z_{1:t}, x_{1:t})$, and that's equivalent to maximizing $p(z_{1:t} |m, x_{1:t})$

<p align="center">
    <img src="https://github.com/RicoJia/The-Dream-Robot/assets/39393023/dc6f0135-7a45-4183-ae9c-f295ae062ba4" alt="image alt text" height="100"/>
</p>

To evaluate that, we go through all $T$, all $N$ beams, and all cells $J$ along a beam, and take the ln() fo the products to transform them into sums

$$
\text{argmax}_{m_j} p(z_{1:t} | m, x_{1:t}) = \sum_{T} \sum_{N} \sum_{J} \alpha_{j} ln (m_j) + \beta_{j} ln(1-m_j)
\\
=> 
m_j = \frac{\alpha_{j}}{\alpha_{j} + \beta_{j}}
$$

$\alpha_{j}$ is the count of reflections at cell $j$, and $\beta_{j}$ is the count of pass-throughs at cell $j$

#### Occupancy Grid Mapping

- Probability is occurence of an event/all outcomes, whereas odds is occurence of an event/occurence of other outcomes. The probability of rolling a dice to 1 is 1/6, but its odds is 1/5. **Odds is another way to achieve probability**

- Log odds: odds is odd to work with: if `odds(win/lose) = 0.6`, then `odds(lose/win) = 1.66`. It's not intuitive to reason about it. However, applying log to it will make it `log(odds(win/lose)) = -log(odds(lose/win))`, which is more intuitive. So, log odds is $log(\frac{P}{1-P})$, aka "logit", or logistic unit function.

- The goal is to evaluate the probability of map given observations, and poses. The map probability is calculated by multiplying all cell possibilities together. As an iterative algorithm, we want to be able to achieve an iterative structure: 
    $$
    p(m|z,x) = \prod_{i} p(m_i | z_{1:t}, x_{1:t}), \text{where a cell occupancy value } m_i \text{ could be 0, 1}
    \\
    ...
    \\
    = \prod_{i} \frac{p(m_{i} | z_t, x_t) p(z_t | x_t)}{p(m_i)} \frac{p(m_i | x_{1:t}, z_{1:t-1})}{p(z_{t} | x_{1:t}, z_{1:t-1})}
    $$

  - This is iterative with $p(m_i | x_{1:t}, z_{1:t-1})$. However, it could be further optimized: 
    1. Probability multiplication is always more prone to underflow
    2. The observation term can be elimiated, through the log odds trick as a common term.

  Therefore, using the simple odds: $p(\neg{m_i} | z_{1:t}, x_{1:t}) = 1 - p(m_i | z_{1:t}, x_{1:t})$
    $$
    \frac{p(m_i | z_{1:t}, x_{1:t})}{p(\neg{m_i} | z_{1:t}, x_{1:t})} = 
        \frac{p(m_i | z_t, x_t)}{p(\neg{m_i} | z_t, x_t)}
        \frac{p(m_i | z_{1:t-1}, x_{1:t-1})}{p(\neg{m_i} | z_{1:t-1}, x_{1:t-1})}
        \frac{p(m_i)}{p(\neg{m_i})}
    $$
  In a more compact form, we note $odds(p) = \frac{p}{1-p}$ after taking the log of the above,
    $$
    ln(odds(p(m_i^t | z_{1:t}, x_{1:t}))) = 
    \\
    = ln(odds(1-p(m_{t-1}^i|z_{t-1}, x_{t-1})))
    + ln(odds(p(m_i | z_t, x_t)))
    + ln(odds(p(m_i)))
    $$

  So $ln(odds(1-p(m_{t-1}^i|z_{t-1}, x_{t-1})))$ is a recursive term that can be achieved from the last iteration; $ln(odds(p(m_i | z_t, x_t)))$ is the "inverse sensor model"; $ln(odds(p(m_i)))$ us a prior prpbablity of the map from $t=0$

Algorithm:
    ```
    For all cells in view,
    l_{t,i} = l_{t-1, i} + inv_sensor_model(mi, xi, zt) - l0
    end for
    ```


### Sensor Model

Sensor Model: $p(z_t|m_{t-1},x_t)$ can be given as distance of endpoint-of-the-beam + Gaussian noise.

- Vanilla: Update about the endpoint model
    - the likelihood of observation Z, given map and x. There are a few possibilities:
        - See Z as a result of Z_truth + Gaussian Sensor noise. Represented as $p_{hit}$
        - See Z as a result of unexpected object in front ot the true obstacle. Note, the closer Z is to the laser scanner, the more likely it could be. This is somewhat "lame?"
        - If z=max, it's likely a failure. it has a non-zero likelihood that you will see this when z=zmax, and x and map indicate that there shouldn't be a failure
        - Random reading: it's likely that Z you see is a random value. We think that likelihood is 1/z_max
    - Total probablity = [z_hit * p_hit + ... z_rand * p_rand]
- But, bmapping is using likelihood fields for range finders. Because of limitations of beam models: Cluttered environment, $p(z|x,m)$ is very unsmooth along x. Slight change in x could result in very different distributions for z.
    1. A likelihood field is a map where each pixel's value is its distance to the nearest obstacle
    2. Upon receiving laser observations, project them on to the map. Then, at each end point, find its distance to the nearest obstacle. That value obeys a Gaussian distribution
    3. There's also a probability for random reading as well.
    4. $p = z1 * N(z, sigma^2) + z_rand/z_max + p(zmax)$

## Important Notes

### Importance sampling

- We have our target trajectory distribution $P(x|z,u)$. However, our particles from the last step is updated using the motion model and odometry. Therefore, they are drawn from a "proposed distribution", $\pi(x|z,u)$. Note, we have the z because we can technically still incorporate that into the estimation.
- So intuitively, if we draw a sample from $\pi$, and know (its apperance in target_distribution)/(its appearance in the current distribution), we can get an idea how likely it will be drawn in the target probablity. So we use this ratio as the "importance weight"
- The importance weight is multiplicative, i.e.

    $$\pi(x_{1:t}|z_{1:t}, u_{1:t-1}) = \pi(x_{t}|x_{1:t-1}, z_{1:t}, u_{1:t-1})\pi(x_{1:t-1}| z_{1:t}, u_{1:t-1}) $$

- Then, the weight can be calculated recursively (for steps, please see the original paper [1])
    $$w_i \alpha \frac{p(z_t|m_{t-1}, x_t)p(x_t|x_{t-1}, u_{t-1})}{\pi(x_t|x_{1:t-1}, z_{1:t}, u_{1:t-1})}w_{t-1}$$

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

## References

[1] Grisetti, G., Stachniss, C., and Burgard, W. 2007. Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters. In Proceedings of the 20th International Joint Conference on Artificial Intelligence (IJCAI'07), January 6–12, 2007, Hyderabad, India. 123–130. DOI:10.1234/567890.1234567.

[2] Guyue's Blog Post on Gmapping (Mandarin)

[3] Murphy, K. 2001. Rao-Blackwellized Particle Filtering for Dynamic Bayesian Networks. In Proceedings of the 17th Conference on Uncertainty in Artificial Intelligence (UAI'01), August 2–5, 2001, Seattle, WA, USA. 202–210. DOI:10.1234/567890.1234568.

[4] https://medium.com/@matthewjacobholliday/an-introduction-to-log-odds-86d364fa6630

[5] Sebastian Thrun, Wolfram Burgard, and Dieter Fox. 2005. Probabilistic Robotics (Intelligent Robotics and Autonomous Agents series). The MIT Press.

[6] Allen, P. 2017. Lecture Notes on ICC Kinematics. COMS W4733 Computational Aspects of Robotics. Columbia University. Available at: https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf.