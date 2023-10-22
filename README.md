# The Dream Robot

Welcome to The Dream Robot Project! Here we aim to build a mini robot management system with two robots. In the future, there could be more! 

This project consists of:
- The Dream Rover (a.k.a coffee bot) 
- The Dream Arm (a.k.a RJJE Arm)
- The Dream Hub - central control wensite


## Setting up a Base
1. `cd dream-setup/setup_mobile_base && ansible-playbook -i hosts.ini configure_base.yml`
2. On host machine, use `sudo arp-scan -l` to find the rpi's IP
3. `ssh-copy-id ricojia@<RPI_IP>`
4. `ssh ricojia@<RPI_IP>`
5. `start_docker` to start container
