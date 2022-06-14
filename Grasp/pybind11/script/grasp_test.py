from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
from cnoid.GraspPlugin import *
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
set_robot(robotItem)
# set target object
set_object(objItem)
# execute grasp
grasp()
print("--- my own program--- ")
from bpbot.utils import print_quaternion
print_quaternion([1,2,3,4])
import timeit
start = timeit.default_timer()

import numpy as np
import bpbot.module_asp.asp_client as aspclt
aspc = aspclt.ASPClient()
imgpath = "/home/hlab/bpbot/data/test/depth3.png"
y = np.array([[4, 5], [6, 7]])

y2bytes=np.ndarray.tobytes(y)

res = aspc.predict(imgpath=imgpath, grasps=y2bytes)
print(res.action, res.graspno)
end = timeit.default_timer()
print("Time cost: ", end-start)
import timeit
start = timeit.default_timer()

import bpbot.driver.phoxi.phoxi_client as pclt
pxc = pclt.PhxClient(host ="127.0.0.1:18300")

pxc.triggerframe()
# pcd = pxc.getpcd()

end = timeit.default_timer()
print("Time cost: ", end-start)
