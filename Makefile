# Build docker project
.PHONY : docker_build
docker_build:
	docker build -t racer .

# Run docker project
.PHONY : docker_run
docker_run:
	docker run -it --network host   --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --device=/dev/video0:/dev/video0 --mount type=bind,source=$(CURDIR),target=/workspace/src --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --entrypoint /bin/bash racer
