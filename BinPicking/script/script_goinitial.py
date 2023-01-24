from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot

print("[*] Move to initial pose! ")

nxt = NxtRobot(host='[::]:15005')
nxt.openHandToolLft()
nxt.openHandToolRgt()
nxt.goInitial()


