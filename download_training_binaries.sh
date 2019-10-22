#!/bin/bash
wget -c https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v0.3.0-linux/AirSim.zip;
mkdir -p /home/$USER/Documents/AirSim;
unzip AirSim.zip;
mv AirSim AirSim_Training;
wget --directory-prefix=AirSim_Training/AirSimExe/Content/Paks -c https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v0.3.0-linux/Building99.pak;
wget --directory-prefix=AirSim_Training/AirSimExe/Content/Paks -c https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v0.3.0-linux/Soccer_Field.pak;
wget --directory-prefix=AirSim_Training/AirSimExe/Content/Paks -c https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v0.3.0-linux/ZhangJiaJie.pak;
wget --directory-prefix=/home/$USER/Documents/AirSim -c https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing/releases/download/v1.0-linux/settings.json;
rm AirSim.zip;