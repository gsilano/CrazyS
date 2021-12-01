import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.interpolate import splprep, splev
from numpy import linalg as LA
import json
import sys


N_points = 1000
u2 = np.linspace(0,1,N_points)
if len(sys.argv) != 2:
    print("Specify the name of the csv file of the path \n" +
          "Should be included in the data_points_files directory")
    exit()
path = sys.argv[1]
traj = pd.read_csv("data_points_files/" + path + ".csv")
# Desired position
x_desired = list(traj["x"])
y_desired = list(traj["y"])
z_desired = list(traj["z"])

# Desired Linear Velocity
vx_desired = list(traj["vx"])
vy_desired = list(traj["vy"])
vz_desired = list(traj["vz"])


# Desired Attitute (roll-yaw-yaw)
roll_desired = list(traj["roll"])
pitch_desired = list(traj["pitch"])
yaw_desired = list(traj["yaw"])
# Desired Attitute (roll-pitch-yaw)
# w_att_desired = list(traj["w_att"])
# x_att_desired = list(traj["x_att"])
# y_att_desired = list(traj["y_att"])
# z_att_desired = list(traj["z_att"])

tck, u = splprep(
    [x_desired,y_desired,z_desired,
    vx_desired,vy_desired,vz_desired,
    roll_desired, pitch_desired, yaw_desired],
    s=10.0,per=0,k=3)

new_points = splev(u2, tck,der=0)

# Desired Acceleration
dt = np.mean(np.divide(np.diff(new_points[0]), new_points[3][:-1]))
ax_desired = np.diff(new_points[3])/dt
ax_desired = np.append(ax_desired, ax_desired[-1])
ay_desired = np.diff(new_points[4])/dt
ay_desired = np.append(ay_desired, ay_desired[-1])
az_desired = np.diff(new_points[5])/dt
az_desired = np.append(az_desired, az_desired[-1])

# Desired Angular Velocity
omegax_desired = np.diff(new_points[6])/dt
omegax_desired = np.append(omegax_desired, omegax_desired[-1])
omegay_desired = np.diff(new_points[7])/dt
omegay_desired = np.append(omegay_desired, omegay_desired[-1])
omegaz_desired = np.diff(new_points[8])/dt
omegaz_desired = np.append(omegaz_desired, omegaz_desired[-1])

# Plot position_original vs spline_position
pos_fig, pos_axs = plt.subplots(2)
pos_fig.suptitle('Position')
pos_axs[0].plot(new_points[0],new_points[1], 'b')
pos_axs[0].plot(x_desired, y_desired, "g")
pos_axs[0].axis('equal')
pos_axs[0].legend({"xy_spline", "xy_original_points"})
pos_axs[1].plot(new_points[2], 'b')
pos_axs[1].plot(z_desired,"g")
pos_axs[1].legend({"z_spline", "z_original_points"})

# Plot velocity_original vs spline_velocity
vel_fig, vel_axs = plt.subplots(2)
vel_fig.suptitle('Velocity')
vel_axs[0].plot(new_points[3],new_points[4], 'b')
vel_axs[0].plot(vx_desired, vy_desired, "g")
vel_axs[0].axis('equal')
vel_axs[0].legend({"vxvy_spline", "vxvy_original_points"})
vel_axs[1].plot(new_points[5], 'b')
vel_axs[1].plot(vz_desired,"g")
vel_axs[1].legend({"vz_spline", "vz_original_points"})

# Plot velocity_original vs spline_velocity
att_fig, att_axs = plt.subplots(3)
att_fig.suptitle('Attitude (roll, pitch, yaw)')
att_axs[0].plot(new_points[6], 'b')
att_axs[0].plot(roll_desired, "g")
att_axs[0].legend({"roll_spline", "roll_original_points"})
att_axs[1].plot(new_points[7], 'b')
att_axs[1].plot(vz_desired,"g")
att_axs[1].legend({"pitch_spline", "pitch_original_points"})
att_axs[2].plot(new_points[8], 'b')
att_axs[2].plot(vz_desired,"g")
att_axs[2].legend({"yaw_spline", "yaw"})

# Plot acceleration
acc_fig, acc_axs = plt.subplots(3)
acc_fig.suptitle('Acceleration')
acc_axs[0].plot(ax_desired)
acc_axs[0].legend({'ax'})
acc_axs[1].plot(ay_desired)
acc_axs[1].legend({'ay'})
acc_axs[2].plot(az_desired)
acc_axs[2].legend({'az'})

# Plot angular rate
omega_fig, omega_axs = plt.subplots(3)
omega_fig.suptitle('Angular velocity')
omega_axs[0].plot(omegax_desired)
omega_axs[0].legend({'omegax'})
omega_axs[1].plot(omegay_desired)
omega_axs[1].legend({'omegay'})
omega_axs[2].plot(omegaz_desired)
omega_axs[2].legend({'omegaz'})

# Write the new points in a json file
df = {"x":new_points[0].tolist(),"y":new_points[1].tolist(),"z":new_points[2].tolist(),
      "vx":new_points[3].tolist(),"vy":new_points[4].tolist(),"vz":new_points[5].tolist(),
      "ax":ax_desired.tolist(),"ay":ay_desired.tolist(),"az":az_desired.tolist(),
      "roll":new_points[6].tolist(),"pitch":new_points[7].tolist(),"yaw":new_points[8].tolist(),
      "omegax":omegax_desired.tolist(),"omegay":omegay_desired.tolist(),"omegaz":omegaz_desired.tolist()}
file = open("spline/"+path+".json","w")
json.dump(df,file)
file.close()

plt.show()
