from argparse import ArgumentParser
import subprocess

class DockerImageBuilder():
    def __init__(self, args):
        self.args = args        

    def build_docker_image(self):
        # if a base image is not specified, we use the Ubuntu 18, CUDA 10 image from NVIDIA
        docker_build_command = ['docker', 'build', '--network=host', \
                            '-t', self.args.target_image, \
                            '-f',  self.args.dockerfile, \
                            '--build-arg', 'BASE_IMAGE=' + self.args.base_image, \
                            '.']

        print(" ".join(docker_build_command))
        subprocess.call(docker_build_command)

def main(args):
    docker_image_builder = DockerImageBuilder(args)
    docker_image_builder.build_docker_image()

if __name__=="__main__":
    parser = ArgumentParser(description='AirSim Neurips-Game-of-Drones docker image builder')
    parser.add_argument('--dockerfile', type=str, default='Dockerfile', help='path to docker file')
    parser.add_argument('--base_image', type=str, default="nvidia/cudagl:10.0-devel-ubuntu18.04", help='base image name AND tag, on top of which the target image is built')
    parser.add_argument('--target_image', type=str, help='desired name of target image name AND tag')

    args = parser.parse_args()

    # if a target image name is not specified, let's call it airsim_neurips:SOURCE_IMAGE_TAG
    if not args.target_image:
        target_image_tag = args.base_image.split(":")[1] 
        args.target_image = 'airsim_neurips' + ':' + target_image_tag

    main(args)