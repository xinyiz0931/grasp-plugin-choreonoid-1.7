from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
#from cnoid.GraspPlugin import *
#topdir = executableTopDirectory()
topdir = "/home/hlab/choreonoid-1.7.0"
# rootItem = RootItem.instance()
rootItem = RootItem.instance

# load robot model
robotItem = BodyItem()
#robotItem.load(topdir + "/ext/graspPlugin/RobotModels/PA10/PA10.yaml")
robotItem.load(topdir+"/ext/graspPlugin/RobotModels/PA10/PA10.yaml")
rootItem.addChildItem(robotItem)
#ItemTreeView.instance().checkItem(robotItem)
ItemTreeView.instance.checkItem(robotItem)
# load object model
objItem = BodyItem()
objItem.load(topdir + "/ext/graspPlugin/Samples/Object/ahiruLowHrp.wrl")
objItem.body.rootLink.setTranslation([0.59, -0.1, 0.55])
rootItem.addChildItem(objItem)
ItemTreeView.instance.checkItem(objItem)

# set grasping robot
# set_robot(robotItem)
#RootItem.setRobot(robotItem)
# set target object
#set_object(objItem)
"""

# execute grasp
grasp()
"""
