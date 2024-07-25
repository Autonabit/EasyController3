# Raspberry Pi Pico Docker SDK

A lightweight SDK environment for Raspberry Pi Pico in a Docker container.

## Pulling the Image from Docker Hub and Running

The latest image is available on [Docker Hub](https://hub.docker.com/repository/docker/lukstep/raspberry-pi-pico-sdk/general)
and can be used to run a container.
The following commands show how to run the container using the Docker Hub image:

```bash
docker run -d -it --name pico-sdk --mount type=bind,source=${PWD},target=/home/dev lukstep/raspberry-pi-pico-sdk:latest
docker exec -it pico-sdk /bin/sh
```

The directory from which the `docker run` command was executed will be mounted in the container at `/home/dev`.
After attaching to the SDK container, you can build your project by executing the following steps:

```bash
cd /home/dev
mkdir build
cd build
cmake .. && make -j4