import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# start timestamp from waypoint generator i.e. when start is pressed
# end timestamp from RELAY: timeoutTimer_.start() at: 1652874184.808009763 then a while(1) is inserted to stop coordination
# mapSize is read from rostopic publification when done: rostopic echo -n1 /UAV0/mapSize
# dist is read from rostopic publification when done: rostopic echo -n1 /UAV0/dist_traversed

df = pd.read_csv('performanceSingle.csv')
df.drop('start', inplace=True, axis=1)
df.drop('end', inplace=True, axis=1)
#print(df)


avg_s, avg_d, avg_t = df.mean()             # Population mean = sum(data)/len(data)
var_s, var_d, var_t = df.var()              # Population variance = sum( (datapoint-population mean)^2 = err from mean )
# ddof=0 affects the set-size (N-ddof) to use when normalising, =0 pop div, =1 sample div
std_s, std_d, std_t = df.std(ddof=0)        # Population diviation = sqrt( population variance )
# From baseline data:
mapsSize_tot = 80284.95

print("Means, mapSize time, dist:                ", round(avg_s,2), "\t", round(avg_d,2), "\t", round(avg_t,2) )
print("Variance, mapsize, time(sec), dist:       ", round(var_s,2), "\t", round(var_d,2), "\t", round(var_t,2) )
print("Std diviation, mapsize, time(sec), dist:  ", round(std_s,2), "\t", round(std_d,2), "\t", round(std_t,2) )
print("mapsize_avg from above divided by mapsSize_tot = {} from baseline test".format(mapsSize_tot))
print("Coverage (%): ", round( 100/mapsSize_tot*round(avg_s,2) ,2) )
print("\n")





# print("MANUALLY - only for understanding/verification")
# # https://stackabuse.com/calculating-variance-and-standard-deviation-in-python/