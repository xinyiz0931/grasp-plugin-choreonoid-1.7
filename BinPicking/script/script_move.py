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
import timeit
import numpy as np

print("[*] Move robot! ")

start = timeit.default_timer()

nxt = NxtRobot(host='[::]:15005')
motion_seq = get_motion()
num_seq = int(len(motion_seq)/20)
motion_seq = np.reshape(motion_seq, (num_seq, 20))
print(f"[*] Total {num_seq-1} motion sequences! ")

# import os
# os.system("bash /home/hlab/bpbot/script/force.sh")


nxt.playMotion(motion_seq)

end = timeit.default_timer()
print("[*] Time: {:.2f}s".format(end - start))

