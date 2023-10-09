# Rpi Configuration List


### Rpi Configuration
1. Initially on rpi:
    1. install raspbian
    2. hostname should be ricojia.
    3. amazon vpc set up

1. On host machine
    ```bash
    ssh-copy-id ricojia@100.66.47.12
    sudo apt install ansible && \
    ```

### Commands (to go into ansible playbook)
```
docker build -t dream_mobile_base .
docker run --name my_ros_container -v /home/ricojia/software/The-Dream-Robot/:/home/The-Dream-Robot --rm -it --network="host" --privileged dream_mobile_base
```

### Software Installation List
1. docker (using curl)
