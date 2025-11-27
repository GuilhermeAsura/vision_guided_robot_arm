from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

for i in range(robot.getNumberOfDevices()):
    device = robot.getDeviceByIndex(i)
    print(device.getName())