# Minimal graspPlugin for Choreonoid 1.7

### Plugins: 
- Grasp
- PRM
- MotionFile
- BinPicking

### Robot and object models
- RobotModels
- Samples

### Requirements
Ubuntu 18, C++ (std+11), CMake > 3.10, Pybind11

### Installation

1. Install Choreonoid 1.7 following the [manual](https://choreonoid.org/ja/documents/1.7/index.html) on the official website. 
2. Check if the main windows can be loaded successfully: 
```
cd choreonoid-1.7.0
bin/choreonoid
```
3. Clone this repository
```
cd choreonoid-1.7.0/ext
git clone https://github.com/xinyiz0931/grasp-plugin-choreonoid-1.7.git
```
4. Install graspPlugin, you can follow manuals from graspPlugin [wiki](http://www.hlab.sys.es.osaka-u.ac.jp/grasp/ja/node/311). Otherwise follow these steps: 
5. 
```
cd choreonoid-1.7.0
./ext/Grasp/install-requiresities-ubuntu.sh
ccmake .
(In ccmake, check the following options) 
(GRASP_PLUGINS  Grasp;PRM;MotionFile;BinPicking)
(USE_PYBIND11   ON)
(USE_PYTHON3    ON)
(USE_QT5        ON)
make
```
6. Test if the installation is successfully done
```
cd choreonoid-1.7.0
bin/chorenoid ext/graspPlugin/BinPicking/project/binpicking.cnoid
```
