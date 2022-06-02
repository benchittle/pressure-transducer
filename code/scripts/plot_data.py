import pandas as pd
from datetime import datetime
import os
from matplotlib import pyplot

INPATH = "/home/benc/Documents/DIY3_out/test run 1"


def read_diy3(path, sensorID):
     # Read data and rename columns, using timestamp as the index.
    data = pd.read_csv(
        filepath_or_buffer=path, 
        index_col="timestamp", 
        parse_dates=True
    )
    data.rename(
        columns={
            "pressure" : sensorID + "_P", 
            "temperature" : sensorID + "_T"}, 
        inplace=True
    )
    
    return data

# For reading RBR's raw engineering output.
def read_rbr(path, sensorID):
    # Read data and rename columns, using timestamp as the index.
    data = pd.read_table(
        filepath_or_buffer=path,
        sep="    ",
        skiprows=31, 
        usecols=[0, 1, 2], # Select Timestamp, Pressure, and Temperature
        names=["timestamp", sensorID + "_T", sensorID + "_P", ], # Rename columns
        encoding="ISO-8859-1", # Default runs into some characters it can't parse
        engine="python"
        ) 
    # Convert time datetime objects
    data["timestamp"] = pd.to_datetime(data["timestamp"], format="%d-%b-%Y %H:%M:%S.000")
    data.set_index("timestamp", inplace=True)
    # Convert pressure to mbar
    data[sensorID + "_P"] *= 100 

    return data

# Read all DIY3 and RBR data in the given directory into a DataFrame.
def read_data(dir_path):
    all_data = {}
    for f in os.listdir(dir_path):
        name, ext = os.path.splitext(f)
        # Read DIY3 data
        if ext == ".csv" and "_" in name:
            sensorID, timestamp = name.split("_")
            #timestamp = datetime.strptime(timestamp, r"%Y%m%d-%H%M")
            data = read_diy3(dir_path + "/" + f, sensorID)
        # Read RBR data
        elif ext == ".txt" and name.endswith("eng"):
            sensorID = "RBR"
            data = read_rbr(dir_path + "/" + f, sensorID)
        else:
            continue
        
        # Add each data file to a list for that sensor
        if sensorID in all_data:
            all_data[sensorID].append(data)
        else:
            all_data[sensorID] = [data]

    # Concatenate each sensor's data into a single dataframe per sensor
    all_data = {id : pd.concat(all_data[id]) for id in all_data}

    # Concatenate all the data into a single dataframe.
    all_data = pd.concat(all_data.values(), axis=1).sort_index()
    all_data = all_data.reindex(sorted(all_data.columns), axis=1)

    return all_data


# Plot all pressure data together
def plot_pressure(all_data):
    all_data[[c for c in all_data.columns if c.endswith("_P")]].plot()
    pyplot.show()


# Plot all pressure data but only show one at a time.
def plot_pressure_individually(all_data):
    for c in all_data.columns:
        if c.endswith("_P"):
            all_data[c].plot()
            pyplot.show()


# Plot all temperature data together.
def plot_temperature(all_data):
    all_data[[c for c in all_data.columns if c.endswith("_T")]].plot()
    pyplot.show()


def main():
    sensor_data = read_data(INPATH)

if __name__ == "__main__":
    main()