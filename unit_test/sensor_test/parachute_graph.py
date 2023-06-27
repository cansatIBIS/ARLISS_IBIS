import pandas as pd
import matplotlib.pyplot as plt
df = pd.read_csv('/Users/fukudakazuya/ARLISS_IBIS/log/log_csv/velocity_check_ver2 2023-06-26 16:25:34.260847.csv',header=None).T
# df.columns = ["v_x","v_y","v_z","v","x","y","z","roll","pitch","yaw"]
df.columns = ["v_x","v_y","v_z","v"]
df[["v_x","v_y","v_z","v"]].plot()
# df[["x"]].plot()
# df[["y"]].plot()
# df[["z"]].plot()
# df[["x","y","z"]].plot(subplots=True)
# df[["roll","pitch","yaw"]].plot(subplots=True)
plt.show()