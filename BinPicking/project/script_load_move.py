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

print("This Script!!!!!")
topdir = executableTopDirectory
root_dir = os.path.join(topdir, "ext/bpbot/")

mf_path = os.path.join(root_dir, "data/motion/motion.dat")
#mf_path = os.path.join(root_dir, "data/motion/calib_r.dat")
start = timeit.default_timer()

plan_success = load_motionfile(mf_path, dual_arm=False)
# plan_success = load_motionfile(mf_path)
print(f"[*] Plannning success? => {plan_success}")
nxt = NxtRobot(host='[::]:15005')
motion_seq = get_motion()
num_seq = int(len(motion_seq)/20)
print(f"[*] Total {num_seq-1} motion sequences! ")
motion_seq = np.reshape(motion_seq, (num_seq, 20))

#nxt.playMotion(motion_seq)

end = timeit.default_timer()
print("[*] Time: {:.2f}s".format(end - start))
