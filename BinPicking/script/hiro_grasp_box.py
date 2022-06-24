from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
from cnoid.GraspPlugin import *
from cnoid.BinPicking import *
topdir = executableTopDirectory
print(topdir)
# rootItem = RootItem.instance()
rootItem = RootItem.instance

# load robot model
robotItem = BodyItem()
# robotItem.load(topdir+"/ext/graspPlugin/RobotModels/PA10/PA10.yaml")
robotItem.load(topdir+"/ext/graspPlugin/RobotModels/HIRO/HIRO_with_provision.yaml")

rootItem.addChildItem(robotItem)
print('---------------- TEST -----------------')
print(robotItem.body.joint(4).q)
#ItemTreeView.instance().checkItem(robotItem)
ItemTreeView.instance.checkItem(robotItem)

# load object model
objItem = BodyItem()
objItem.load(topdir + "/ext/graspPlugin/Samples/Object/M0Hrp.wrl")
# objItem.load(topdir + "/ext/graspPlugin/Samples/Object/ahiruLowHrp.wrl")
objItem.body.rootLink.setTranslation([0.50, 0.20, 0.20])
rootItem.addChildItem(objItem)
ItemTreeView.instance.checkItem(objItem)

# set grasping robot
set_robot(robotItem, arm_id=1)
# set target object
set_object(objItem)
# execute grasp
grasp()
# plan()
