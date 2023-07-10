
from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
from cnoid.GraspPlugin import *
from cnoid.BinPicking import *

from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot
# mf_path = "/home/hlab/bpbot/data/calibration/20230220/motion.txt"
# load_motionfile(mf_path)
# ARM = "left"
# best_grasp_wrist= [0.581,0.135,0.109,-112.5,-90.0,90.0]
# best_grasp_wrist= [0.531,0.135,0.129,-112.5,-90.0,90.0]
# PLACE_XYZ = [0.48,0.26,0.2]

# success = plan_binpicking(ARM, best_grasp_wrist[:3], best_grasp_wrist[3:], PLACE_XYZ )
motion_seq = get_motion()
nxt = NxtRobot(host='[::]:15005')
print(f"Move robot! Total {motion_seq.shape[0]} motion sequences! ")

# nxt.playMotion(motion_seq)
old_lhand = "STANDBY"
old_rhand = "STANDBY"
for m in motion_seq:
    if (m[-2:] != 0).all(): lhand = "OPEN"
    else: lhand = "CLOSE"
    if (m[-4:-2] != 0).all(): rhand = "OPEN"
    else: rhand = "CLOSE"
    if old_rhand != rhand:
        if rhand == "OPEN": nxt.openHandToolRgt()
        elif rhand == "CLOSE": nxt.closeHandToolRgt()
    if old_lhand != lhand:
        if lhand == "OPEN": nxt.openHandToolLft()
        elif lhand == "CLOSE": nxt.closeHandToolLft()
    old_lhand = lhand
    old_rhand = rhand
    nxt.setJointAngles(m[3:], tm=m[0])

