# Drones

This project aims to drive simulate drone usage for warehouse inspection.

## Build docker container

On a terminal, in the current folder, run the following.


```
# Build Docker image
docker build -t robotti-build-tool .
# Generate a container using the image
docker-compose run dev-container

```

docker rmi $(docker images -q)
docker rm $(docker ps -aq)
