import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
# df = pd.read_csv('/Users/fukudakazuya/ARLISS_IBIS/log/log_csv/velocity_check_ver2 2023-08-03 23:13:35.451566.csv',header=None).T
df = pd.read_csv('/Users/matsushimakouta/Documents/ARLISS/ARLISS_IBIS/log/log_csv/velocity_check_ver2 2023-08-03 23:13:35.451566.csv',header=None).T
v_x_lst=[]
v_y_lst=[]
v_z_lst=[]
v_lst=[]
t = np.arange(0, 2.4, 0.12)
for i in range(85,105):
    v_x_lst.append(df.loc[i,0])
    v_y_lst.append(df.loc[i,1])
    v_z_lst.append(df.loc[i,2])
    v_lst.append(df.loc[i,3])
c1,c2,c3,c4 = "blue","green","red","black"      # 各プロットの色
l1,l2,l3,l4 = "v_x","v_y","v_z","v"   # 各ラベル
fig, ax = plt.subplots()
ax.set_xlabel('t')  # x軸ラベル
ax.set_ylabel('velocity')  # y軸ラベル
# ax.set_aspect('equal') # スケールを揃える
ax.grid()            # 罫線
#ax.set_xlim([-10, 10]) # x方向の描画範囲を指定
#ax.set_ylim([0, 1])    # y方向の描画範囲を指定
ax.plot(t,v_x_lst, color=c1, label=l1)
ax.plot(t,v_y_lst, color=c2, label=l2)
ax.plot(t,v_z_lst, color=c3, label=l3)
ax.plot(t,v_lst, color=c4, label=l4)
ax.legend(loc=0)    # 凡例
fig.tight_layout()  # レイアウトの設定
# plt.savefig('hoge.png') # 画像の保存
plt.show()