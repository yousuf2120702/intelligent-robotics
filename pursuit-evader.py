import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def go_to_goal(x, y, yaw, goal_x, goal_y):
    k = 1
    desired_angle = np.arctan2(goal_y - y, goal_x - x)
    # desired_angle = (2*np.pi + desired_angle if desired_angle < 0 else desired_angle)
    alpha = desired_angle - yaw
    print(alpha*180/np.pi)
    # if alpha > np.pi:
    #     print("true1")
    #     alpha = np.pi - alpha
    # if alpha < -np.pi:
    #     print("true2")
    #     alpha = -np.pi - alpha
    w = 2 * k * alpha
    return w

arena = [50, 50]

evader_speed = 0
pursuer_speed = 2


evader_x = np.random.uniform(0, 50)
evader_y = np.random.uniform(0, 50)
evader_yaw = np.random.uniform(-np.pi, np.pi)
evader_pos = [[evader_x, evader_y, evader_yaw]]

pursuer_x = np.random.uniform(0, 50)
pursuer_y = np.random.uniform(0, 50)
pursuer_yaw = np.random.uniform(-np.pi, np.pi)
pursuer_pos = [[pursuer_x, pursuer_y, pursuer_yaw]]

steps = 0
done = False

while not done:
    print(steps)
    w = go_to_goal(pursuer_x, pursuer_y, pursuer_yaw, evader_x, evader_y)
    pursuer_yaw += w*0.1
    if pursuer_yaw < -np.pi:
        pursuer_yaw = p.pi - (np.abs(pursuer_yaw) - np.pi)
    if pursuer_yaw > np.pi:
        pursuer_yaw = -np.pi + (pursuer_yaw - np.pi)

    pursuer_x += np.sin(pursuer_yaw)*pursuer_speed*0.1
    pursuer_y += np.cos(pursuer_yaw)*pursuer_speed*0.1
    flag = False
    while flag == False:
        evader_w = np.random.uniform(-np.pi, np.pi)
        evader_yaw += evader_w*0.1
        evader_x_temp = evader_x + np.sin(evader_yaw)*evader_speed*0.1
        evader_y_temp = evader_y + np.cos(evader_yaw)*evader_speed*0.1
        if 0<evader_x_temp<50 and 0<evader_y_temp<50:
            flag=True
            evader_x = evader_x_temp
            evader_y = evader_y_temp
    evader_pos.append([evader_x, evader_y, evader_yaw])
    pursuer_pos.append([pursuer_x, pursuer_y, pursuer_yaw])
    steps += 1
    if np.abs(np.hypot(evader_x-pursuer_x, evader_y-pursuer_y))<0.5 or steps==1000:
        break

plt.rcParams["figure.figsize"] = [10, 10]
plt.rcParams["figure.autolayout"] = True

fig, ax = plt.subplots()
marker_size = 50

def animate(i):
   fig.clear()
   ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(0, 50), ylim=(0, 50))
   ax.set_xlim(0, 50)
   ax.set_ylim(0, 50)
   e = ax.scatter(evader_pos[i][0], evader_pos[i][1], s=marker_size, cmap="RdBu_r", marker="o", edgecolor='black')
   p = ax.scatter(pursuer_pos[i][0], pursuer_pos[i][1], s=marker_size, cmap="RdBu_r", marker="o", edgecolor='blue')

plt.grid(b=None)
ani = animation.FuncAnimation(fig, animate, interval=100, frames=range(steps))

ani.save('animation.gif', writer='pillow')