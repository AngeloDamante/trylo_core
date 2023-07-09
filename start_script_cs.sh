#!/bin/bash

echo "[ BUILDING ]"
colcon build ~/trylo_core/src
souurce ~/trylo_core/install/setup.bash
echo "[ DONE ]"

echo -e "${RED}---------------------------------------------------------------${NC}"
echo -e "${RED}        _     _                               _             _  ${NC}"
echo -e "${RED} __   _(_)___(_) ___  _ __     ___ ___  _ __ | |_ _ __ ___ | | ${NC}"
echo -e "${RED} \ \ / / / __| |/ _ \| '_ \   / __/ _ \| '_ \| __| '__/ _ \| | ${NC}"
echo -e "${RED}  \ V /| \__ \ | (_) | | | | | (_| (_) | | | | |_| | | (_) | | ${NC}"
echo -e "${RED}   \_/ |_|___/_|\___/|_| |_|  \___\___/|_| |_|\__|_|  \___/|_| ${NC}"
echo -e "${RED}---------------------------------------------------------------${NC}"

ros2 launch trylo_launch vision_control.xml