import os
from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
from cnoid.GraspPlugin import *
from cnoid.BinPicking import *

from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot

import timeit
import numpy as np

topdir = executableTopDirectory
root_dir = os.path.join(topdir, "ext/bpbot/")
mf_path = os.path.join(root_dir, "data/motion/motion.dat")

start = timeit.default_timer()

plan_success = load_motionfile(mf_path)
if plan_success:
    nxt = NxtRobot(host='[::]:15005')
    motion_seq = get_motion()
    num_seq = int(len(motion_seq)/21)
    print(f"Total {num_seq} motion sequences! ")
    motion_seq = np.reshape(motion_seq, (num_seq, 21))
    # for m in motion_seq:
    #     if m[1] == 0: 
    #         nxt.closeHandToolLft()
    #     elif m[1] == 1:
    #         nxt.openHandToolLft()
    #     nxt.setJointAngles(m[2:21],tm=m[0]) # no hand open-close control
    print("Finish! ")
else: 
    print("Planning failed! ")
# ======================= Record the data ===================s=========

end = timeit.default_timer()
print("Time: {:.2f}s".format(end - start))
