# %%
import pandas as pd
from datetime import datetime
import os
from matplotlib import pyplot

INPATH = "/home/benc/Documents/DIY3_out/C401-E15D"

allData = {}

def read_diy3(path, sensorID):
     # Read data and rename columns, using timestamp as the index.
    data = pd.read_csv(
        filepath_or_buffer=path, 
        index_col="timestamp", 
        parse_dates=True
    )
    data.rename(columns={"pressure" : sensorID + "_P", "temperature" : sensorID + "_T"}, inplace=True)
    return data

def read_rbr(path, sensorID):
    # Read data and rename columns, using timestamp as the index.
    data = pd.read_excel(
        io=path,
        skiprows=5, 
        usecols=[0, 1, 2], # Select Timestamp, Pressure, and Temperature
        names=["timestamp", sensorID + "_T", sensorID + "_P", ]) # Rename columns
    # Convert time datetime objects
    data["timestamp"] = pd.to_datetime(data["timestamp"], dayfirst=True)
    data.set_index("timestamp", inplace=True)
    # Convert pressure to mbar
    data[sensorID + "_P"] *= 100 

    return data

# %%
for f in os.listdir(INPATH):
    name, ext = os.path.splitext(f)
    # Read DIY3 data
    if ext == ".csv" and "_" in name:
        sensorID, timestamp = name.split("_")# Select Timestamp and Pressure columns
        #timestamp = datetime.strptime(timestamp, r"%Y%m%d-%H%M")
        data = read_diy3(INPATH + "/" + f, sensorID)
    # Read RBR data
    elif ext == ".xls":
        sensorID = "RBR"
        data = read_rbr(INPATH + "/" + f, sensorID)
    else:
        continue
    
    # Add each data file to a list for that sensor
    if sensorID in allData:
        allData[sensorID].append(data)
    else:
        allData[sensorID] = [data]

# Concatenate each sensor's data into a single dataframe per sensor
allData = {sensor : pd.concat(allData[sensor]) for sensor in allData}

# Concatenate all the data into a single dataframe.
allData = pd.concat(allData.values(), axis=1).sort_index()
allData = allData.reindex(sorted(allData.columns), axis=1)
allData[[c for c in allData.columns if c.endswith("_P")]].plot()
#pyplot.show()

# %%
#temp = allData.copy()
#temp["RBR_P"] = temp["RBR_P"].fillna(method="ffill")
#temp[[c for c in temp.columns if c.endswith("_P")]].plot(rot=45)
#allData.to_csv(f"{INPATH}/sampleData.csv")
for c in allData.columns:
    if c.endswith("_P"):
        allData[c].plot()
        print(c)
        pyplot.show()





# %%
