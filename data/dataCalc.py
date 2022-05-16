import matplotlib.pyplot as plt
import numpy as np
import pandas as pd



df = pd.read_csv('data.csv')
# this forces the column's data into time variables
#df['time'] = pd.to_datetime(df['time'], infer_datetime_format=True)
# Convert to time-deltas
df['time'] = pd.to_timedelta(df['time'])


avg_d, avg_t = df.mean()
std_d, std_t = df.std()

print(df)
print("Means, time, dist:         ", round(avg_d,2), '\t', avg_t )
print("Std diviation, time, dist: ", round(std_d,2), '\t', std_t )


# Convert to seconds
# df['time'] = df['time'].dt.total_seconds()
# avg_t, avg_d = df.mean()
# std_t, std_d = df.std()
# print("Std diviation, time, dist: ", round(std_t,2), round(std_d,2) )
# print("Means, time, dist:         ", round(avg_t,2), round(avg_d,2) )