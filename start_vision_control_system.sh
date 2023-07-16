#!/bin/bash
RED='\033[0;31m'
NC='\033[0m' 

echo -e "${RED}---------------------------------------------------------------${NC}"
echo -e "${RED}        _     _                               _             _  ${NC}"
echo -e "${RED} __   _(_)___(_) ___  _ __     ___ ___  _ __ | |_ _ __ ___ | | ${NC}"
echo -e "${RED} \ \ / / / __| |/ _ \| '_ \   / __/ _ \| '_ \| __| '__/ _ \| | ${NC}"
echo -e "${RED}  \ V /| \__ \ | (_) | | | | | (_| (_) | | | | |_| | | (_) | | ${NC}"
echo -e "${RED}   \_/ |_|___/_|\___/|_| |_|  \___\___/|_| |_|\__|_|  \___/|_| ${NC}"
echo -e "${RED}---------------------------------------------------------------${NC}"

echo "[ BUILDING ]"
source ~/trylo_venv/bin/activate 
cd ~/trylo_core/
python3 power_on.py --build "no"
echo "[ DONE ]"

cd ~/trylo_core
source install/setup.bash
ros2 launch trylo_launch vision_control.xml