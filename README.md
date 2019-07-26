# attention_tracking

## Intro
The attention tracking function is achieved by utilizing OpenFace( https://github.com/TadasBaltrusaitis/OpenFace ) face pose estimation, Get Face Pose w.r.t. head_neck system and PBVS method, which we control the robot by inverse kinematic derivation w.r.t. desired attention position in the world coordinate reference.
serial and sprotocol related files are created by Duo (https://www.linkedin.com/in/duoluthedeveloper/, who is a Ph.D. student in CIDSE, ASU).

## Install

### Prerequisite
  1. [OpenFace](https://github.com/TadasBaltrusaitis/OpenFace).
  
  ```shell
  $ git clone https://github.com/TadasBaltrusaitis/OpenFace.git
  $ git checkout OpenFace_v2.0.1
  ```
   Then follow the installation steps [here](https://github.com/TadasBaltrusaitis/OpenFace/wiki/Unix-Installation).
  ```shell
  $ OPENFACE=<SourcePATH> # Configure as the root path of the cloned repo.
  ```
  2. Opencv 3.2.0.
  
### Compile
  ```shell
  $ mkdir build; cd build
  $ cmake -DOpenFace_DIR=$OPENFACE/install/lib/cmake/OpenFace/ \
CMAKE_BUILD_TYPE=RELEASE  -DCMAKE_CXX_FLAGS="-std=c++11" \
-DCMAKE_EXE_LINKER_FLAGS="-std=c++11"  ../
```

## Run
* Make sure the left camera is connected first and recognized as webcam(1).
```shell
# Running attention tracking demo
$ ./at -device 1 -cam_width 640 -cam_height 480 -fx 446.596289 -fy 444.697260 -cx 300.222605 -cy 213.31848
# Reading servo log
$ ./readservo
```
