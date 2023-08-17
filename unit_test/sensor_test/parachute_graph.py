import pandas as pd
import matplotlib.pyplot as plt
df = pd.read_csv('/Users/fukudakazuya/Downloads/velocity_check_ver2 2023-08-11 14_08_31.326017.csv',header=None).T
df.columns = ["v_x","v_y","v_z","v","x","y","z","roll","pitch","yaw","forward_a","right_a","down_a","acc"]
# df.columns = ["v_x","v_y","v_z","v"]
df[["v_x","v_y","v_z","v"]].plot()
df[["x","y","z"]].plot(subplots=True)
df[["roll","pitch","yaw"]].plot(subplots=True)
df[["forward_a","right_a","down_a","acc"]].plot(subplots=True)
plt.show()