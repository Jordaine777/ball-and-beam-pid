import pybullet as p
import pybullet_data
import time
import math

# PID gains
Kp = 50
Ki = 0
Kd = 5

integral = 0
previous_error = 0

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane = p.loadURDF("plane.urdf")
pendulum = p.loadURDF("pendulum.urdf" , basePosition = [0,0,0.1])

p.setJointMotorControl2(
    bodyIndex = pendulum,
    jointIndex = 0,
    controlMode = p.VELOCITY_CONTROL,
    force = 0
)

target_angle = 0
dt = 1/240

for i in range(10000):
    joint_state = p.getJointState(pendulum, 0)
    angle = joint_state[0]
    angular_velocity = joint_state[1]
    
    error = target_angle - angle

    integral += error * dt
    derivative = (error - previous_error) / dt
    control = Kp * error + Ki * integral + Kd * derivative

    p.setJointMotorControl2(
        bodyIndex = pendulum,
        jointIndex = 0,
        controlMode = p.TORQUE_CONTROL,
        force = control
    )
    
    previous_error = error
    p.stepSimulation()
    time.sleep(dt)
