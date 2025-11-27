from controller import Robot
import math


robot = Robot()


timestep = int(robot.getBasicTimeStep())


keyboard = Keyboard()
keyboard.enable(timestep)


joint_names = [
    'shoulder_pan_joint',
    'shoulder_lift_joint', 
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]


joints = []
joints_index_keys = []#Preencher com os atalhos escolhidos para cada junta
joints_movement_keys = []


for name in joint_names:
    joint = robot.getDevice(name)
    joints.append(joint)

current_joint = 0  


home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def move_to_position(position):
    for i, joint in enumerate(joints):
        joint.setPosition(position[i])


def show_positions():
    positions = [joint.getTargetPosition() for joint in joints]
    print("Posições atuais:")
    for i, pos in enumerate(positions):
        print(f"  Junta {i+1}: {pos:.3f} rad")





def getkeyboardkey():

    key = None

    return key




while robot.step(timestep) != -1:
    

    key = getkeyboardkey()

    if key in joints_index_keys:
        current_joint = None #preencher

    elif key in joints_movement_keys:
        if key == None:#preencher
            rad = 0.3
        else:
            rad = -0.3
        


        current_pos = joints[current_joint].getTargetPosition()
        joints[current_joint].setPosition(current_pos + rad)

        
    