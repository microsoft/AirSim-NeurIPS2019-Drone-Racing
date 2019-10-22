#!/bin/bash
wget -c https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v1.0-linux/AirSim.zip;
mkdir -p /home/$USER/Documents/AirSim;
unzip AirSim.zip;
mv AirSim AirSim_Qualification;
wget --directory-prefix=AirSim_Qualification/AirSimExe/Content/Paks -c https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v1.0-linux/Qual_Tier_1_and_Tier_3.pak;
wget --directory-prefix=AirSim_Qualification/AirSimExe/Content/Paks -c https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v1.0-linux/Qual_Tier_2.pak;
wget --directory-prefix=/home/$USER/Documents/AirSim -c https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v1.0-linux/settings.json;
rm AirSim.zip;