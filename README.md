# Minimal graspPlugin for Choreonoid 1.7

## Plugins: 
- Grasp
- PRM
- MotionFile
- BinPicking

## Robot and object models
- RobotModels
- Samples

## Requirements
Ubuntu 18, C++ (std+11), CMake > 3.10, Pybind11

## Installation

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
7. Test if the python script can be loaded
```
cd choreonoid-1.7.0
bin/choreonoid -p ext/graspPlugin/BinPicking/script/pa10_grasp_plan_ahiru.py
(or)
bin/choreonoid ext/graspPlugin/BinPicking/project/python_pa10_grasp_plan_ahiru.cnoid
```
Click the `play` button to check if the motion is planned

## Usage

### Structures

See `ext/graspPlugin/BinPicking/project/`, if a `.cnoid` project's name starts with `python_`, then it is a project containing a python script in `ext/graspPlugin/BinPicking/script/`. The python script which it will execute has a similar name as itself but starts with . For example, 

Project: `ext/graspPlugin/BinPicking/project/python_pa10_grasp_plan_ahiru.cnoid`

Script: `ext/graspPlugin/BinPicking/script/script_pa10_grasp_plan_ahiru.py`

In this case, you can execute the python-script-inside project via: 
```
bin/choreonoid ext/graspPlugin/BinPicking/project/python_pa10_grasp_plan_ahiru.cnoid
```

Also, `ext/graspPlugin/BinPicking/project/`, if a `.cnoid` project's name doesn't containing `python_`, then it can be directly executed via: 
```
bin/choreonoid -p ext/graspPlugin/BinPicking/script/pa10_grasp_plan_ahiru.py
```

### BinPicking plugin functions (NEW)

I revised the motion planning part. Two functions can be used

- load_motionfile()
- get_motion()

```
plan_success = load_motionfile(mf_path)
# plan_success is 0 or 1

motion_seq = get_motion()
num_seq = int(len(motion_seq)/21)
motion_seq = np.reshape(motion_seq, (num_seq, 21))
# motion_seq is a list that can be reshaped to a (Nx21) array, same with the old `motion_ik.dat`
# Reshaped motion_seq can be execute on the robot
```
### Using your own python3.x in Choreonoid 1.7+ 

Especially using anaconda python environment 

1. Enter your conda environment and remove cache files
```
conda activate xxx
cd choreonoid-1.7.0 
rm CMakeCache.txt
rm -r lib/choreonoid-1.7/*
```

2. Open `choreonoid-1.7.0/CMakeLists.txt`, revise contents from line 350
```
if(USE_PYTHON3)
  set(Python_ADDITIONAL_VERSIONS 3.7 3.6 3.5 3.4)
  find_package(PythonLibs 3 REQUIRED)
↓↓ (CMake < 3.12:)
if(USE_PYTHON3)
  set(Python_ADDITIONAL_VERSIONS 3.9 3.8 3.7 3.6 3.5 3.4)
  find_package(PythonInterp 3 REQUIRED)
  find_package(PythonLibs 3 REQUIRED)
↓↓ (CMake < 3.12:)
  find_package(Python 3.9 EXACT REQUIRED COMPOENNTS Interpreter Development)
```

3. Start compile, since we delete the cache file, we need to add the necesasry plugin again
```
ccmake .
(In ccmake, check the following options) 
(GRASP_PLUGINS  Grasp;PRM;MotionFile;BinPicking)
make
```
After these operation, you can keep use this python envrionment. In this case, just keep `CMakeCache.txt`. 
