# Contents of ros_env_setup.sh
sudo_ros_preserve_env(){
    # when passing args from one func to another, use $@ expansion
    local cmd="$@"
    # -E preserves all current user's env variables.
    sudo -E /bin/bash -c "source ${WORKDIRECTORY}/devel/setup.bash; $cmd"
}

