import matplotlib.pyplot as plt
import numpy as np
import pandas as pd



df = pd.read_csv('officeSingle.csv')
# this forces the column's data into time variables
#df['time'] = pd.to_datetime(df['time'], infer_datetime_format=True)
# Convert to time-deltas
df['time'] = pd.to_timedelta(df['time'])


# calculate and print in timestamp notation
# avg_d, avg_t = df.mean()
# std_d, std_t = df.std()
# print(df)
# print("Means, time, dist:         ", round(avg_d,2), '\t', avg_t )
# print("Std diviation, time, dist: ", round(std_d,2), '\t', std_t )


# Convert to seconds
df['time'] = df['time'].dt.total_seconds()
avg_c, avg_t, avg_d = df.mean()
std_c, std_t, std_d = df.std()
var_c, var_t, var_d = df.var()

print(df)
# print("Means, coverage time, dist:               ", avg_c, "\t", avg_t, "\t", avg_d)
# print("Std diviation, coverage, time(sec), dist: ", std_c, "\t", std_t, "\t", std_d)
print("Means, coverage time, dist:                ", round(avg_c,2), "\t", round(avg_t,2), "\t", round(avg_d,2) )
print("Std diviation, coverage, time(sec), dist:  ", round(std_c,2), "\t", round(std_t,2), "\t", round(std_d,2) )
print("Variance, coverage, time(sec), dist:       ", round(var_c,2), "\t", round(var_t,2), "\t", round(var_d,2) )