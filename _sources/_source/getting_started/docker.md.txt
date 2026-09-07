# Docker basics

Docker is optional for the ROS 2 exercises, but it is useful when a tool should
run without installing all of its dependencies on the host computer. This
repository also uses Docker Compose to export the workshop slides without
installing Node.js locally.

Follow the official [Docker Engine installation instructions for
Ubuntu](https://docs.docker.com/engine/install/ubuntu/). Complete Docker's
post-installation steps if workshop commands should run without `sudo`.
Membership in the `docker` group grants elevated privileges, so only enable it
on a computer where that access is appropriate.

## Core concepts

| Term | Meaning |
| --- | --- |
| Image | A read-only template containing an application and its dependencies |
| Container | A running or stopped instance of an image |
| Dockerfile | Instructions for building an image |
| Compose file | Configuration for one or more related containers |
| Volume | Data or a directory mounted into a container |

## Verify Docker

```bash
docker --version
docker compose version
docker run --rm hello-world
```

The final command downloads a small test image, runs it, and removes the
container after it exits.

## Images

Download an image:

```bash
docker pull ubuntu:24.04
```

List local images:

```bash
docker images
```

Build an image from a Dockerfile in the current directory:

```bash
docker build -t my_image .
```

## Containers

Start an interactive Ubuntu container:

```bash
docker run --rm -it ubuntu:24.04
```

The `--rm` option removes the container when it exits.

Start and name a container:

```bash
docker run -it --name my_container ubuntu:24.04
```

List running containers or all containers:

```bash
docker ps
docker ps -a
```

Start an existing stopped container and open a shell:

```bash
docker start my_container
docker exec -it my_container bash
```

Stop and remove it:

```bash
docker stop my_container
docker rm my_container
```

## Docker Compose

Compose reads a `compose.yaml` file and manages its services together.

```bash
docker compose up
docker compose ps
docker compose logs
docker compose down
```

Run a one-off service and remove its container afterward:

```bash
docker compose run --rm SERVICE_NAME
```

Rebuild service images before starting:

```bash
docker compose up --build
```

## Exercise

1. Run an interactive `ubuntu:24.04` container.
2. Inside the container, display the Ubuntu release information.
3. Exit the container and confirm that it was removed.

```bash
docker run --rm -it ubuntu:24.04
cat /etc/os-release
exit
docker ps -a
```


## Using ROS 2 with Docker

Tutorial material: https://github.com/ipa-may/docker_ros2_tutorial


