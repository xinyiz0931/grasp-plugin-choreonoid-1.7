from cnoid.Util import *
from cnoid.Base import *
from cnoid.BodyPlugin import *
from cnoid.GraspPlugin import *
from cnoid.BinPicking import *

rootItem = RootItem.instance

# set grasping robot
robotItem = rootItem.find("PA10")
set_robot(robotItem)

# set target object
objItem = rootItem.find("ahiru")
set_object(objItem)

# execute grasp
grasp()

# execute motion planning
plan()