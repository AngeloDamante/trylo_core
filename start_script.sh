#!/bin/bash
RED='\033[0;31m'
NC='\033[0m' 

echo -e "${RED}---------------------------------${NC}"
echo -e "${RED}---- _____           _       ----${NC}"
echo -e "${RED}----|_   _| __ _   _| | ___  ----${NC}"
echo -e "${RED}----  | || '__| | | | |/ _ \ ----${NC}"
echo -e "${RED}----  | || |  | |_| | | (_) |----${NC}"
echo -e "${RED}----  |_||_|   \__, |_|\___/ ----${NC}"
echo -e "${RED}----           |___/         ----${NC}"
echo -e "${RED}---------------------------------${NC}"

echo "[ GPIO ACTIVATING ]"
echo -e "gogo" | sudo -S chmod go+rw /dev/gpiomem
echo -e "gogo" | sudo -S chmod go+rw /dev/i2c-1
echo "[ DONE ]"

echo "[ TRYLO INITIALISING ]"
source ~/trylo_venv/bin/activate 
python3 ~/trylo_core/start_trylo.py
echo "[ DONE ]"
