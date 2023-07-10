"""
motion_seq.shape: (N,20)
motion_seq[0]     - time
motion_seq[1:2]   - torso
motion_seq[2:4]   - head
motion_seq[4:10]  - rarm
motion_seq[10:16] - larm

motion_seq[16:18] - rhand
motion_seq[18:20] - lhand
"""
from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
from cnoid.GraspPlugin import *
from cnoid.BinPicking import *

from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot
import sys
import time
import numpy as np
from bpbot.device import DynPickClient

# define 
nxt = NxtRobot(host='[::]:15005')
dc = DynPickClient()

mtm = 0.1
fz_thld = 2

# planning motion for grasping
print("Grasping! ")
grasp_mf = "/home/hlab/bpbot/data/motion/monitering.dat"
grasp_success = load_motionfile(grasp_mf, dual_arm=False)
if grasp_success[1] == False: 
    print(f"[!] Approaching the target failed! ")

print(f"[*] Motion planning succeed? ==> {grasp_success.count(True) == len(grasp_success)}")

if grasp_success.count(True) != len(grasp_success):
    sys.exit()

grasp_seq = get_motion()
grasp_seq_num = int(len(grasp_seq)/20)
grasp_seq = np.reshape(grasp_seq, (grasp_seq_num, 20))
print(f"[*] Total {grasp_seq_num-1} motion sequences! ")


nxt.playMotion(grasp_seq[:-1])

print(f"With force monitering! ")
m = grasp_seq[-1]
dc.save()
nxt.setJointAngles(m[1:], tm=m[0], wait=False)
for i in np.arange(0, m[0], mtm):
    fz = dc.get()[2]
    if (fz >  fz_thld and i > m[0]/2) or fz>=4: 
        print("Stopping the motion! Fz=%.3f" % fz)
        nxt.stopMotion()
        break
    time.sleep(mtm)
time.sleep(2)
dc.save_ok()

# load regrasp motion
print(f"regrasp motion! ")
regrasp_mf = "/home/hlab/bpbot/data/motion/motion.dat"
regrasp_success = load_motionfile(regrasp_mf, dual_arm=True)

print(f"[*] Motion planning succeed? ==> {regrasp_success.count(True) == len(regrasp_success)}")

# if regrasp_success.count(True) != len(regrasp_success):
#     sys.exit()

regrasp_seq = get_motion()
regrasp_seq_num = int(len(regrasp_seq)/20)
regrasp_seq = np.reshape(regrasp_seq, (regrasp_seq_num, 20))

nxt.playMotion(regrasp_seq)

# # load fling motion
# print(f"Fling motion! ")
# fling_mf = "/home/hlab/bpbot/data/motion/fling.dat"
# fling_success = load_motionfile(fling_mf, dual_arm=True)

# print(f"[*] Motion planning succeed? ==> {fling_success.count(True) == len(fling_success)}")

# # if fling_success.count(True) != len(fling_success):
# #     sys.exit()

# fling_seq = get_motion()
# fling_seq_num = int(len(fling_seq)/20)
# fling_seq = np.reshape(fling_seq, (fling_seq_num, 20))

# nxt.playMotion(fling_seq)

# load place motion
print(f"Place motion! ")
# place_mf = "/home/hlab/bpbot/data/motion/place.dat"
# place_success = load_motionfile(place_mf, dual_arm=False)

print(f"[*] Motion planning succeed? ==> {place_success.count(True) == len(place_success)}")

# if place_success.count(True) != len(place_success):
#     sys.exit()

place_seq = get_motion()
place_seq_num = int(len(place_seq)/20)
place_seq = np.reshape(place_seq, (place_seq_num, 20))

nxt.playMotion(place_seq)



# from new script
# from cnoid.Util import *
# from cnoid.Base import *
# from cnoid.Body import *
# from cnoid.BodyPlugin import *
# from cnoid.GraspPlugin import *
# from cnoid.BinPicking import *
# import time
# import numpy as np

# PLAY = False
# # initialize

# from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot
# from bpbot.device import DynPickClient
# nxt = NxtRobot(host='[::]:15005')
# dc = DynPickClient()
# # mtm = 0.1
# # fz_thld = 2

# print("[*] Picking motion planning! ")
# plan_pick(xyz=[0.501, 0.052, 0.180], rpy=[90,-90,-90], init=True, clear=False)
# motion_seq = get_motion()
# print(f"[*] Total {motion_seq.shape[0]} motion sequences! ")
# if PLAY: 
#     nxt.playMotion(motion_seq)

# # print("[*] Picking motion planning2! ")
# # plan_pick(xyz=[0.470, -0.152, 0.150], rpy=[120,-90,-90], init=False, clear=True)
# # motion_seq = get_motion()
# # print(f"[*] Total {motion_seq.shape[0]} motion sequences! ")
# time.sleep(4)

# print("[*] Regrasping motion planning! ")
# plan_regrasp(clear=True)
# motion_seq = get_motion()
# print(f"[*] Total {motion_seq.shape[0]} motion sequences! ")
# if PLAY:
#     nxt.playMotion(motion_seq)

# # # nxt.goInitialArm("larm", tm=3)

