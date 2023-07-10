import urx
import sys
import json

from src.gripper import Robotiq_Two_Finger_Gripper
from src.gripper import RobotiqScript
from robot_move import move

def move_func1():
    i = 153
    gripper.self_define_gripper(i)
    while i < 230: # because the physical close status is around 230
        with open('./config/signal.json', 'r') as f:
            data = json.load(f)
            flag = data['signal']
        print(flag)
        gripper.self_define_gripper(i)
        if flag == False and i < 230:
            i+=10
            continue
        elif flag == False and i >=200:
            print("no object detected")
            rob.close()
            sys.exit()
        else:
            print("object detected")
            break

def move_func2():
    gripper.init_gripper()
    count = 1 
    loop = 10
    while count <= loop:
        with open('./config/signal.json', 'r') as f:
            data = json.load(f)
            flag = data['signal']
        print(flag)
        if flag == False:
            gripper.close_gripper()
            count += 1
            continue
        else:
            print("object detected")
            break
    if count >= loop:
        print("no object detected")
        rob.close()
        sys.exit()


if __name__ == "__main__":
    rob = urx.Robot("192.168.1.102")
    gripper = Robotiq_Two_Finger_Gripper(rob,1.3)

    urscript = RobotiqScript(socket_host="127.0.0.1", socket_port=63352, socket_name="gripper_socket")

    gripper.open_gripper()

    # move_func1()
    move_func2()

    action = move()
    action.initial_pose()
    action.square_move_example()
    rob.close()
    sys.exit()