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
from bpbot.device import FTSensor
import timeit
import numpy as np
from multiprocessing import Process

print("[*] Move robot! ")

nxt = NxtRobot(host='[::]:15005')

motion_seq = get_motion()
num_seq = int(len(motion_seq)/20)
motion_seq = np.reshape(motion_seq, (num_seq, 20))
# motion_seq = np.loadtxt('/home/hlab/bpbot/data/motion/motion_ik.dat')
# num_seq = motion_seq.shape[0]
print(f"[*] Total {num_seq-1} motion sequences! ")

############### Record for the whole process
import os
os.system("bash /home/hlab/bpbot/script/start_ft.sh")
nxt.playMotionFT(motion_seq)
os.system("bash /home/hlab/bpbot/script/stop_ft.sh")

ft = FTSensor()
from bpbot.motion import FlingActor
actor = FlingActor()
t_thld = 0.2
r_min = 45
r_max = 80
# ft.plot_file()
fz,tx,ty = ft.plot_interval()
############### Record for one interval
# nxt._playMotionFT(motion_seq)
