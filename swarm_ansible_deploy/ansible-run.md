############################################################
# SWARM DEPLOYMENT – BASIC SSH + ANSIBLE COMMANDS
# Minimal essentials only
############################################################


############################################################
# 1) INSTALL ANSIBLE (ON YOUR DESKTOP)
############################################################

sudo apt update
# Update package list

sudo apt install ansible -y
# Install Ansible

ansible --version
# Confirm Ansible is installed



############################################################
# 2) GENERATE SSH KEY (ON YOUR DESKTOP)
############################################################

ssh-keygen
# Create SSH key pair (press Enter 3 times for defaults)

ls ~/.ssh
# Verify key files exist (id_rsa or id_ed25519)



############################################################
# 3) COPY SSH KEY TO ROBOT (FIRST TIME ONLY)
############################################################

ssh-copy-id robomaster1@192.168.1.50
# Replace IP if needed
# This installs your public key on the robot



############################################################
# 4) TEST SSH LOGIN (SHOULD NOT ASK PASSWORD AFTER THIS)
############################################################

ssh robomaster1@192.168.1.50
# If login works without password, SSH key is working

exit
# Exit robot



############################################################
# 5) TEST ANSIBLE CONNECTION
############################################################

ansible -i inventory.ini robots -m ping
# Test connection to all robots in inventory
# Expected output: "pong"



############################################################
# 6) INSTALL DOCKER COLLECTION (ON DESKTOP)
############################################################

ansible-galaxy collection install community.docker
# Required for docker_container module



############################################################
# 7) RUN DEPLOYMENT PLAYBOOK
############################################################

ansible-playbook -i inventory.ini deploy.yml -K
# -K asks for sudo password on robot
# This deploys everything



############################################################
# 8) CHECK RUNNING CONTAINER (MANUAL CHECK)
############################################################

ssh robomaster1@192.168.1.50
# Connect to robot

docker ps
# See running containers

exit



############################################################
# 9) IF SSH HOST KEY WARNING APPEARS
############################################################

ssh-keygen -R 192.168.1.50
# Remove old fingerprint if IP changed



############################################################
# 10) IF YOU REFLASH ROBOT (SSH BROKEN)
############################################################

ssh-copy-id robomaster1@192.168.1.50
# Reinstall SSH key



############################################################
# THAT’S IT.
# These are the only commands you need for:
# - SSH setup
# - Ansible testing
# - Swarm deployment
############################################################
