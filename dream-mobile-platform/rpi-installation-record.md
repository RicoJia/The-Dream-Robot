# Rpi Configuration List

### Software Installation List
1. docker (using curl)

### Commands (to go into ansible playbook)
```
docker build -t dream_mobile_base .
docker run --name my_ros_container -v /home/ricojia/software/The-Dream-Robot/:/home/The-Dream-Robot --rm -it --network="host" --privileged dream_mobile_base
```
