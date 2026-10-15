import matplotlib.pyplot as plt

#goal is to write my own PID that I can plot and gain an intuition of how each term behaves

#state variables
position = 0
velocity = 0
dt = .01
mass = 1
target = 100
previous_error = 0
error_sum = 0

k_p = 10
k_i = 0.5
k_d = 4

position_list = []


for i in range(2000):
    #controller
    error = target - position
    proportional = error * k_p
    integral = k_i * error_sum + error * dt
    derivative = k_d * (error - previous_error) / dt
    force = proportional + derivative + integral
    #print( proportional, derivative, force)
    saturated = False

    if force > 20:
        force = 20
        saturated = True
    if force < -20:
        force = -20
        saturated = True


    #plant: physics controlling the car
    acceleration = force / mass - 10
    velocity = velocity + acceleration * dt
    position = position + velocity * dt

    position_list.append(position)
    previous_error = error


    if saturated == False:
        error_sum = error_sum + error * dt

plt.plot(position_list)
plt.show()

