
from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
from cnoid.GraspPlugin import *
from cnoid.BinPicking import *

from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot
motion_seq = get_motion()
nxt = NxtRobot(host='[::]:15005')
print(f"Move robot! Total {motion_seq.shape[0]} motion sequences! ")

nxt.playBinPickingMotion(motion_seq)

