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

mf_path = "/home/hlab/bpbot/data/motion/motion.dat"
plan_success = load_motionfile(mf_path)
print(f"[*] Motion planning succeed? ==> {plan_success}")
if plan_success.count(True) == len(plan_success):
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
    ft.plot_interval()
    




    # t_thld = 0.1
    # from bpbot.motion import FlingActor
    # actor = FlingActor()
    # r3_min = 50
    # r3_max = 80
    # r4_min = 30
    # r4_max = 60
    # fz,tx,ty = ft.plot_interval()
    # print("fz=",fz,"tx=",tx,"ty=",ty)
    # if fz > 6:
    #     print("[-] Put back! ")
    #     actor.add_place_action(dest="front")
    # elif fz < 0.85 and fz > 0.70:
    #     print("[-] Good! ")
    #     actor.add_place_action(dest="right") 
    # else:
    #     print("[-] Fling! ")
    #     j3 = -np.sign(ty)*((np.abs(ty)/t_thld) * (r3_max-r3_min) + r3_min)
    #     j4 = ((np.abs(tx)/t_thld) * (r4_max-r4_min) + r4_min)
    #     # j4 = 0 if tx > 0 else ((tx/t_thld) * (r4_max-r4_min) + r4_min)
    #     # j4 = ((np.abs(tx)/t_thld) * (r_max-r_min) + r_min)
    #     if fz > 4: 
    #         h = 0.38
    #     else:
    #         h= 0.48
    #     print("fling: j3=",j3, "j4=",j4)
    #     actor.add_fling_action(j3=j3, j4=j4, h=h)
    # import matplotlib.pyplot as plt
    # plt.show()
    # from datetime import datetime as dt
    # print("[*] Save results")
    # tdatetime = dt.now()
    # tstr = tdatetime.strftime('%Y%m%d%H%M%S')
    # save_dir = "/home/hlab/Desktop/exp_ft" 
    # tstr_dir = os.path.join(save_dir, tstr)
    # if not os.path.exists(save_dir): os.mkdir(save_dir)
    # if not os.path.exists(tstr_dir): os.mkdir(tstr_dir)
    # import shutil
    # shutil.copyfile("/home/hlab/bpbot/data/depth/depth_cropped.png", os.path.join(tstr_dir, "depth.png"))
    # shutil.copyfile("/home/hlab/bpbot/data/depth/result.png", os.path.join(tstr_dir, "grasp.png"))
    # shutil.copyfile("/home/hlab/bpbot/data/force/out.txt", os.path.join(tstr_dir, "force.txt"))
    # shutil.copyfile("/home/hlab/bpbot/data/force/out.png", os.path.join(tstr_dir, "force.png"))

