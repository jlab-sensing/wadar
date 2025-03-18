# BeagleBone Black environment

This container was created for standardized testing of digital signal processing functions and compilation of radar functions. 

```bash
sudo docker build -t chipotle .
```

```bash
sudo docker run -it chipotle
```

Anything compiled can be extracted from the container using,

```bash
docker ps
docker cp CONTAINER_ID:/home/wadar/02_uwb/FlatEarth/c_code/frameLogger .
```