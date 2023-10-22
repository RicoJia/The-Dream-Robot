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
