import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys


# start timestamp from waypoint generator i.e. when start is pressed
# end timestamp from RELAY: timeoutTimer_.start() at: 1652874184.808009763 then a while(1) is inserted to stop coordination
# mapSize is read from rostopic publification when done: rostopic echo -n1 /UAV0/mapSize
# dist is read from rostopic publification when done: rostopic echo -n1 /UAV0/dist_traversed

file = str(sys.argv[1])
#print(file)
df = pd.read_csv(file)
#print(df.head())



# Hack to insert missing . in ros-times
def insert_dot(string):
    return string[:10] + '.' + string[10:]

df['rosbagTimestamp'] = df['rosbagTimestamp'].astype(str)
df['rosbagTimestamp'] = df['rosbagTimestamp'].apply(insert_dot)
df['rosbagTimestamp'] = df['rosbagTimestamp'].astype(float)





# Set start-time = 0
startTime = df['rosbagTimestamp'].iloc[0]
#print(startTime)
df['rosbagTimestamp'] = df['rosbagTimestamp'] - startTime
#print(df.head())




# Plotting
time = df['rosbagTimestamp']
data = df['data']

# To set correct y-label, check parsed filename
if "dist" in file:
    yData = "Distance(meters)"
else:
    yData = "Map size(nr. of points)"



fig, ax = plt.subplots()
ax.plot(time, data)
ax.set_title(file)
ax.set_xlabel( "Time(seconds)" )
ax.set_ylabel( yData )

def annot_max(x,y, ax=None):
    xmax = x[np.argmax(y)]
    ymax = y.max()
    text= "Maximum: {:.3f}".format(ymax)
    if not ax:
        ax=plt.gca()
    bbox_props = dict(boxstyle="square,pad=0.3", fc="w", ec="k", lw=0.72)
    arrowprops=dict(arrowstyle="->",connectionstyle="angle,angleA=0,angleB=60")
    kw = dict(xycoords='data',textcoords="axes fraction",
              arrowprops=arrowprops, bbox=bbox_props, ha="right", va="top")
    ax.annotate(text, xy=(xmax, ymax), xytext=(0.9,0.6), **kw)

annot_max(time, data)

plt.show()
