import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# Mapsize is found ROS_WARN print of point_cloud_own_pcd_
# Time is found by manual stopwatch
# dist is read from coordination print

df = pd.read_csv('baselineSingle.csv')
# this forces the column's data into time variables
#df['time'] = pd.to_datetime(df['time'], infer_datetime_format=True)
# Convert to time-deltas
df['time'] = pd.to_timedelta(df['time'])



# Convert to seconds
df['time'] = df['time'].dt.total_seconds()
avg_s, avg_d, avg_t = df.mean()             # Population mean = sum(data)/len(data)
var_s, var_d, var_t = df.var()              # Population variance = sum( (datapoint-population mean)^2 = err from mean )
# ddof=0 affects the set-size (N-ddof) to use when normalising, =0 pop div, =1 sample div
std_s, std_d, std_t = df.std(ddof=0)        # Population diviation = sqrt( population variance )

# print(df)
# print("Means, mapsize time, dist:               ", avg_s, "\t", avg_d, "\t", avg_t)
# print("Std diviation, mapsize, time(sec), dist: ", std_s, "\t", std_d, "\t", std_t)
print("Means, mapSize time, dist:                ", round(avg_s,2), "\t", round(avg_d,2), "\t", round(avg_t,2) )
print("Variance, mapsize, time(sec), dist:       ", round(var_s,2), "\t", round(var_d,2), "\t", round(var_t,2) )
print("Std diviation, mapsize, time(sec), dist:  ", round(std_s,2), "\t", round(std_d,2), "\t", round(std_t,2) )
print("\n")





# MANUALLY - only for understanding/verification
# https://stackabuse.com/calculating-variance-and-standard-deviation-in-python/
cov  = df[df.columns[0]].to_numpy()
dist = df[df.columns[1]].to_numpy()
time = df[df.columns[2]].to_numpy()
# print(cov, time, dist)

# Mean = mu
mu_dist = np.mean(dist) 
print("mean_dist", mu_dist)

# Variance = rho2 = sum of spread from mean
#                           (must square to remove neg. value)
rho2 = sum(1/(len(dist)-1)*(dist-mu_dist)**2)      # Bessels correction = sample variance
#rho2 = sum( ((dist-mu_dist)**2)/len(dist) )         # un-corrected = population variance
print( "Variance = rho2", rho2 )
# variance is a measure of how far individual (numeric) values in a dataset are from the mean
# A high variance tells us that the values in our dataset are far from their mean.
# low variance tells us that the values are quite close to the mean

# population std. deviation = rho = spread within data
rho = np.sqrt(rho2)
print( "population std. deviation", rho)
# Low values of standard deviation tell us that individual values are closer to the mean.
# High values, on the other hand, tell us that individual observations are far away from the mean of the data.
# Unlike variance, the standard deviation will be expressed in the same units of the original observations.


# This plot would be nice
# https://mathbitsnotebook.com/Algebra1/StatisticsData/STSD.html