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
print(f"[*] Total {num_seq-1} motion sequences! ")
motion_seq = np.reshape(motion_seq, (num_seq, 20))

nxt.playMotionSeq(motion_seq)

end = timeit.default_timer()
print("[*] Time: {:.2f}s".format(end - start))
