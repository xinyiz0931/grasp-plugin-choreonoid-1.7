import os
import random

from cnoid.Util import *
from cnoid.Base import *
from cnoid.Body import *
from cnoid.BodyPlugin import *
from cnoid.GraspPlugin import *
from cnoid.BinPicking import *

from bpbot.binpicking import *
from bpbot.config import BinConfig
from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot
import bpbot.driver.phoxi.phoxi_client as pclt

import timeit
import cv2
import numpy as np
from bpbot.utils import main_proc_print, warning_print, notice_print

start = timeit.default_timer()

# ========================== define path =============================
topdir = executableTopDirectory

# plan_motion()
# load_file()
# get root dir
#root_dir = os.path.abspath("./")
root_dir = os.path.join(topdir, "ext/bpbot/")
#root_dir = os.path.realpath(os.path.join(os.path.realpath(__file__), "../../"))
main_proc_print(f"Start at {root_dir} ")

depth_dir = os.path.join(root_dir, "data/depth/")

# pc_path = os.path.join(root_dir, "data/pointcloud/out.ply")
img_path = os.path.join(root_dir, "data/depth/depth.png")
# img_path = os.path.join(root_dir, "data/test/depth9.png")
crop_path = os.path.join(root_dir, "data/depth/depth_cropped.png")
config_path = os.path.join(root_dir, "cfg/config.yaml")
calib_path = os.path.join(root_dir, "data/calibration/calibmat.txt")
mf_path = os.path.join(root_dir, "data/motion/motion.dat")
traj_path = os.path.join(root_dir, "data/motion/calib_ik.dat")
draw_path = os.path.join(root_dir, "data/depth/result.png")

cfg = BinConfig(config_path)

# ======================= get config info ============================

bincfg = BinConfig(config_path)
cfg = bincfg.config
# =======================  generate motion ===========================

main_proc_print("Execute! ")
point_clouds = []
texture_images = []

nxt = NxtRobot(host='[::]:15005')

motion_seq = np.loadtxt(traj_path)
for i, m in enumerate(motion_seq):
    if m[1] == 0: 
        nxt.closeHandToolLft()
    elif m[1] == 1:
        nxt.openHandToolLft()
    nxt.setJointAngles(m[2:27],tm=m[0]) # no hand open-close control
    pxc = pclt.PhxClient(host="127.0.0.1:18300")
    pxc.triggerframe()
    pc = pxc.getpcd()
    gs = pxc.getgrayscaleimg()
    pc_name = f"/home/hlab/pc_{i}.npy"
    gs_name = f"/home/hlab/gs_{i}.png"
    np.save(pc_name, pc)
    cv2.imwrite(gs_name, gs)
    notice_print("capture done! ")


main_proc_print("Finish! ")

end = timeit.default_timer()
main_proc_print("Time: {:.2f}s".format(end - start))