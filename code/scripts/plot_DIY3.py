import pandas as pd
from datetime import datetime
import os
from matplotlib import pyplot

INPATH = "/home/benc/Documents/DIY3_out/C401-E15D"

allData = {}


for f in os.listdir(INPATH):
    name, ext = os.path.splitext(f)
    if ext != ".csv":
        continue
    
    sensorID, timestamp = name.split("_")
    timestamp = datetime.strptime(timestamp, r"%Y%m%d-%H%M")

    # Read just the pressure data and use timestamp as the index
    data = pd.read_csv(INPATH + "/" + f, index_col="timestamp")
    # Rename pressure columns to be unique to each sensor
    data.rename(columns={"pressure":sensorID + "_P", "temperature":sensorID + "_T"}, inplace=True)
    
    # Add each data file to a list for that sensor
    if sensorID in allData:
        allData[sensorID].append((timestamp, data))
    else:
        allData[sensorID] = [(timestamp, data)]

# Sort each sensor's data (kinda)
for key, lst in allData.items():
    lst.sort(key=lambda x: x[0])
    allData[key] = [x[1] for x in lst]

# Concatenate each sensor's data into a single dataframe per sensor
allData = {sensor : pd.concat(allData[sensor]) for sensor in allData}

# Concatenate all the data into a single dataframe.
allData = pd.concat(allData.values(), axis=1).sort_index()
allData = allData.reindex(sorted(allData.columns), axis=1)
print(allData)


allData[[c for c in allData.columns if c.endswith("_P")]].plot(rot=45)
allData.to_csv(f"{INPATH}/sampleData.csv")
pyplot.show()




