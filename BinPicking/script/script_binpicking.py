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

import timeit
import numpy as np
start = timeit.default_timer()

# ========================== define path =============================
topdir = executableTopDirectory

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
traj_path = os.path.join(root_dir, "data/motion/motion_ik.dat")
draw_path = os.path.join(root_dir, "data/depth/result.png")

cfg = BinConfig(config_path)

# ======================= get config info ============================

bincfg = BinConfig(config_path)
cfg = bincfg.config

# ======================== get depth img =============================

point_array = get_point_cloud(depth_dir, cfg['pick']['distance'],
                                 cfg['width'],cfg['height'])
# # pcd = o3d.io.read_point_cloud("./data/test/out.ply")
#     # point_array = pcd.points

# =======================  compute grasp =============================

if point_array is not None: 
    grasps, img_input = detect_grasp_point(n_grasp=10, 
                                    img_path=img_path, 
                                    margins=cfg['pick']['margin'],
                                    g_params=cfg['graspability'],
                                    h_params=cfg['gripper'])
    cv2.imwrite(crop_path, img_input)
else: grasps = None

# =======================  picking policy ===========================
if grasps is None:
    best_action = -1
    best_graspno = 0
    rx,ry,rz,ra = np.zeros(4)
else:

    if cfg['exp_mode'] == 0:
        # 0 -> graspaiblity
        best_grasp = grasps[0]
        best_graspno = 0
        best_action = 0 

    elif cfg['exp_mode'] == 1: 
        # 1 -> proposed circuclar picking
        grasp_pixels = np.array(grasps)[:, 1:3]
        best_action, best_graspno = predict_action_grasp(grasp_pixels, crop_path)
        best_grasp = grasps[best_graspno]

    elif cfg['exp_mode'] == 2:
        # 2 -> random circular picking
        best_grasp = grasps[0]
        best_action = random.sample(list(range(6)),1)[0]

    (rx,ry,rz,ra) = transform_image_to_robot((best_grasp[1],best_grasp[2],best_grasp[4]),
                    img_path, calib_path, point_array, cfg["pick"]["margin"])

# draw grasp 
    img_grasp = draw_grasps(grasps, crop_path,  cfg['gripper'], top_idx=best_graspno, color=(73,192,236), top_color=(0,255,0))
    cv2.imwrite(draw_path, img_grasp)

# =======================  generate motion ===========================

    gen_success = generate_motion(mf_path, [rx,ry,rz,ra], best_action)
    plan_success = load_motionfile(mf_path)
    # if gen_success and plan_success:
    if plan_success:
        nxt = NxtRobot(host='[::]:15005')
        motion_seq = get_motion()
        num_seq = int(len(motion_seq)/21)
        print(f"Total {num_seq} motion sequences! ")
        motion_seq = np.reshape(motion_seq, (num_seq, 21))
        for m in motion_seq:
            if m[1] == 0: 
                nxt.closeHandToolLft()
            elif m[1] == 1:
                nxt.openHandToolLft()
            nxt.setJointAngles(m[2:21],tm=m[0]) # no hand open-close control
        main_proc_print("Finish! ")
        
    else: 
        warning_print("Planning failed! ")
    # ======================= Record the data ===================s=========
    main_proc_print("Save the results! ")

# if success_flag:
#     tdatetime = dt.now()
#     tstr = tdatetime.strftime('%Y%m%d%H%M%S')
#     input_name = "{}_{}_{}_{}_.png".format(tstr,best_grasp[1],best_grasp[2],best_action)
#     cv2.imwrite('./exp/{}'.format(input_name), input_img)
#     cv2.imwrite('./exp/draw/{}'.format(input_name), cimg)
#     cv2.imwrite('./exp/full/{}'.format(input_name), full_image)

end = timeit.default_timer()
main_proc_print("Time: {:.2f}s".format(end - start))
