# !bin/bash

# **Usage**
# - for running default image, training binaries, in windowed mode:    
#     `$ ./run_docker_image.sh "" training` 
# - for running default image, qualification binaries, in windowed mode:    
#     `$ ./run_docker_image.sh "" qualification` 
# - for running default image, training binaries, in headless mode:    
#     `$ ./run_docker_image.sh "" training headless`
# - for running default image, qualification binaries, in headless mode:    
#     `$ ./run_docker_image.sh "" qualification headless`
# - for running a custom image in windowed mode, pass in you image name and tag:    
#     `# $ ./run_docker_image.sh DOCKER_IMAGE_NAME:TAG`
# - for running a custom image in headless mode, pass in you image name and tag, followed by "headless":    
#     # $ ./run_docker_image.sh DOCKER_IMAGE_NAME:TAG headless

# This script takes three optional arguments

# 1st argument is the name (and tag) of the dockerfile to run
# by default, it is set to "airsim_neurips:10.0-devel-ubuntu18.04"
# else user can specify a docker image as follows:
# $ ./run_docker_image.sh DOCKER_IMAGE_NAME:TAG
DOCKER_IMAGE_NAME=${1:-airsim_neurips:10.0-devel-ubuntu18.04}

# 2nd argument: can be "training" or "qualification"
TRAINING_OR_QUALIFICATION=${2:-training}

# 3rd argument: if user passes "headless", binary runs in headless mode
IS_HEADLESS=${3:-notheadless}

# this block is for running X apps in docker
XAUTH=/tmp/.docker.xauth
if [[ ! -f $XAUTH ]]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# per use the following commented out code for different options
if [[ $2 = "training" ]]; then 
    UNREAL_BINARY_COMMAND="bash /home/airsim_user/AirSim_Training/AirSimExe.sh -windowed -opengl"
elif [[ $2 = "qualification" ]]; then 
    UNREAL_BINARY_COMMAND="bash /home/airsim_user/AirSim_Qualification/AirSimExe.sh -windowed -opengl"
fi

# eleminate terminal output and run airsim process in the background
# UNREAL_BINARY_COMMAND="bash /home/airsim_user/AirSim_Training/AirSimExe.sh -windowed -opengl &>/dev/null &"

# set window resolution
# UNREAL_BINARY_COMMAND="/home/airsim_user/AirSim_Training -windowed -ResX=1080 -ResY=720"

# now, let's check if we need to run in headless mode or not
# set SDL_VIDEODRIVER_VALUE to '' if windowed mode, 'offscreen' if headless mode
SDL_VIDEODRIVER_VALUE='';
if [[ $3 = "headless" ]]; then 
    SDL_VIDEODRIVER_VALUE='offscreen';
fi

# now, set the environment varible SDL_VIDEODRIVER to SDL_VIDEODRIVER_VALUE
# and tell the docker container to execute UNREAL_BINARY_COMMAND
nvidia-docker run -it \
    -e SDL_VIDEODRIVER=$SDL_VIDEODRIVER_VALUE \
    -e SDL_HINT_CUDA_DEVICE='0' \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --runtime=nvidia \
    --rm \
    $DOCKER_IMAGE_NAME \
    /bin/bash -c "$UNREAL_BINARY_COMMAND"