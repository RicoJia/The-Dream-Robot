# Contents of ros_env_setup.sh
sudo_ros_preserve_env(){
    local cmd="$@"
    sudo -E /bin/bash -c "source ${WORKDIRECTORY}/devel/setup.bash; $cmd"
}
