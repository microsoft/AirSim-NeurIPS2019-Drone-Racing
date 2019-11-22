#!/bin/bash
wget -c https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v1.1-linux/AirSim.zip;
mkdir -p /home/$USER/Documents/AirSim;
unzip AirSim.zip;
mv AirSim AirSim_Final_Round;
wget --directory-prefix=AirSim_Final_Round/AirSimExe/Content/Paks -c https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v1.1-linux/Final_Tier_1_and_Tier_2.pak;
wget --directory-prefix=AirSim_Final_Round/AirSimExe/Content/Paks -c https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v1.1-linux/Final_Tier_3.pak;
wget --directory-prefix=/home/$USER/Documents/AirSim -c https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v1.1-linux/settings.json;
rm AirSim.zip;