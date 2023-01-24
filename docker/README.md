# Docker setup (Windows)

## Prerequisites

Docker desktop, WSL2, VcXsrv Windows X Server

## Installation

1. Build docker image
```
docker build -t robot-image .
```

2. Run container 
```
docker run -it --name robot-container -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=graphics --gpus all robot-image bash
```


