from controller import Robot, Keyboard
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


for name in joint_names:
    joint = robot.getDevice(name)
    joints.append(joint)
    print(joint)

current_joint = 0  
joint_step = 0.1   


home_position = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
ready_position = [0.0, -1.0, 1.0, -1.0, -1.57, 0.0]


def move_to_position(position):
    for i, joint in enumerate(joints):
        joint.setPosition(position[i])


def show_positions():
    positions = [joint.getTargetPosition() for joint in joints]
    print("Posições atuais:")
    for i, pos in enumerate(positions):
        print(f"  Junta {i+1}: {pos:.3f} rad")


move_to_position(home_position)
print("Movido para posição HOME")


while robot.step(timestep) != -1:
    
    key = keyboard.getKey()
    
    if key == Keyboard.UP:

        current_pos = joints[current_joint].getTargetPosition()
        joints[current_joint].setPosition(current_pos + joint_step)
        print(f"Junta {current_joint + 1}: {current_pos + joint_step:.3f}")
        
    elif key == Keyboard.DOWN:

        current_pos = joints[current_joint].getTargetPosition()
        joints[current_joint].setPosition(current_pos - joint_step)
        print(f"Junta {current_joint + 1}: {current_pos - joint_step:.3f}")
        
    elif key >= ord('1') and key <= ord('6'):

        current_joint = key - ord('1') 
        print(f"Controlando junta {current_joint + 1}")
        
