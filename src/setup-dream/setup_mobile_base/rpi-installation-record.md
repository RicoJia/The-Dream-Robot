## How to SSH
- On robot: sudo openvpn --config profile-4.ovpn. Authname
- On computer
    - https://3.22.234.161:943
    - https://3.22.234.161:943/admin. 

- How to provision
    1. Update address of Rpi in `hosts.ini`
    2. `ansible-playbook -i hosts.ini configure_base.yml -vvv`

