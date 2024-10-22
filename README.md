# The Dream Robot

Welcome to The Dream Robot Project! Here we aim to build a mini robot management system with two robots. In the future, there could be more! 

This project consists of:
- The Dream Mobile Base (a.k.a coffee bot)
- The Dream Arm (a.k.a RJJE Arm)
- The Dream Robot Control Center (web monitoring tools)

To better document and share my development rationale and gotchas, I created a [Medium blog post series](https://ricoruotongjia.medium.com/build-a-robot-monitoring-system-from-scratch-546bea7de730)

## Setting Up The Pi
1. `cd dream-setup/setup_mobile_base && ansible-playbook -i hosts.ini configure_base.yml`
2. On host machine, use `sudo arp-scan -l` to find the rpi's IP
3. `ssh-copy-id ricojia@<RPI_IP>`
4. `ssh ricojia@<RPI_IP>`
5. `start_docker` to start container

## Mobile Base
Please see [here for more details](src/dream_mobile_platform/README.md)

## Component Selection And Comparison

### 3D Lidar - Unitree L1

| Unitree L1       | Livox Mid-360                                              |
|----------------|------------------------------------------------------|
|  USB compatible  | M12 12-pin connector. No PoE allowed, must use a splitter | 
| requires external power | ? |
| 21600 pts/s Data Rate | ? |
| `unitree_lidar_ros`    | Package:  [livox ros driver](https://github.com/Livox-SDK/livox_ros_driver2)                         |
| Noise **L1:** ± 2cm, 8mm resolution        |                           |
| **FoV (Field of View)** L1: 90° vertical  |  Horizontal: 360°, Vertical: -7°~52°                                                |
| [L1 Demo is decent](https://www.youtube.com/watch?v=reCvmW_2ZDQ&pp=ygUKVW5pdHJlZSBMMQ%3D%3D)  |  [Mid360 demo is good](https://www.youtube.com/watch?v=hGayFuhnf1w)                                       |

- PoE is "power over ethernet". Amazon, Roboshop, and DigiKey do not have better alternatives. So the [DJI official website is the best bet](https://store.dji.com/product/livox-three-wire-aviation-connector?vid=117441)

### Power (Purchase After Robot)

- [Laptop Powerbank](https://www.amazon.com/INIU-20000mAh-Portable-Charging-3-Output/dp/B0CB1DTFT6/ref=sr_1_13?crid=Q6LF84663C9U&dib=eyJ2IjoiMSJ9.EfJkIcM3kbSCMMbEQ-MJbj2NZddjKnWaHkJPRLX7Cea8PwJNnGzMZ5QlzSjsr-BbiKkvXFAK4UcgQXfOnMtygiifEDq4Vu2L3-hb-03Dnl5qZk6nUZhekQm7O0CKwOgGZmVX1DrmZyR0D9RGPt7lJb8yl1Q-MpC0mLyNFOg9aIZAlZPM0HOZhOD0R9PRUwOaDcGOX5HR8zhfxBlrZm0D8Pf2vIcb1lut3qxYfTsHPXE.Wk1DaFtXJxdJ6RoJ4v-cVOJpgZqtXLiIf-ly3XAB1kw&dib_tag=se&keywords=12v%2Bpowerbank&qid=1729609994&sprefix=12v%2Bpowerbank%2Caps%2C156&sr=8-13&th=1)
- USBC-12v DC Charger for Nvidia Nano / LiDAR
- USBC-USBA converter (5V, 3A for rpi 4B+)