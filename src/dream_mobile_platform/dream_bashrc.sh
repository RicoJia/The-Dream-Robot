# Contents of ros_env_setup.sh
sudo_ros_preserve_env(){
    # when passing args from one func to another, use $@ expansion
    local cmd="$@"
    # -E preserves all current user's env variables.
    # - `source` vs `./`: `./` will createa a copy of the env of the current session for the new script. 
    # when the script is done, the new env is gone. `source` and `.` will execute the script right on the spot
    sudo -E /bin/bash -c "source ${WORKDIRECTORY}/devel/setup.bash; $cmd"
}

print_opening_msg(){
# NO LEADING spacing or tabs
echo -e "$(cat << 'EOF'
\e[31mHola dreamer - this is a friendly message you might want to know
- We are skipping catkin-building some offline packages. Check out their CMakeLists.txt\e[0m
EOF
)"
}

print_opening_msg


