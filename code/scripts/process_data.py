# %%
import pandas as pd
from datetime import datetime
import os
from matplotlib import pyplot


def read_raw()

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
def read_rbr_eng(path, sensorID) -> pd.DataFrame:
    # Read data and rename columns, using timestamp as the index.
    data = pd.read_table(
        filepath_or_buffer=path,
        sep="    ",
        skiprows=31, 
        usecols=[0, 1, 2], # Select Timestamp, Pressure, and Temperature
        names=["timestamp", sensorID + "_T", sensorID + "_P", ], # Rename columns
        encoding="ISO-8859-1", # Default runs into some characters it can't parse
        engine="python") 
    # Convert time datetime objects
    data["timestamp"] = pd.to_datetime(data["timestamp"], format="%d-%b-%Y %H:%M:%S.000")
    data.set_index("timestamp", inplace=True)
    # Convert pressure to mbar
    data[sensorID + "_P"] *= 100 

    return data


# For reading RBR's legacy xls output.
def read_rbr_xls(path, sensorID) -> pd.DataFrame:
    data = pd.read_excel(
        path, 
        skiprows=5, 
        usecols=[0, 1, 2],
        names=["timestamp", sensorID + "_T", sensorID + "_P"])
    # Parse datetimes.
    data["timestamp"] = pd.to_datetime(data["timestamp"], format="%d/%m/%Y %H:%M:%S.000")
    data.set_index("timestamp", inplace=True)
    # Convert pressure to mbar
    data[sensorID + "_P"] *= 100

    return data


# Read all DIY3 and RBR data in the given directory into a DataFrame.
def read_data(dir_path, trim_ends: int | pd.DataFrame) -> pd.DataFrame:
    all_data = {}
    for f in os.listdir(dir_path):
        name, ext = os.path.splitext(f)
        # Read DIY3 data
        if ext == ".csv" and "_" in name:
            sensorID, timestamp = name.split("_")
            print(f"Reading {name}")
            if name == "S12_20220914-0000":
                print("here")
            #timestamp = datetime.strptime(timestamp, r"%Y%m%d-%H%M")
            data = read_diy3(os.path.join(dir_path, f), sensorID)
        # Read RBR engineering output data
        elif ext == ".txt" and name.endswith("eng"):
            sensorID = "RBR"
            data = read_rbr_eng(os.path.join(dir_path, f), sensorID)
        elif ext == ".xls":
            sensorID = "RBR"
            data = read_rbr_xls(os.path.join(dir_path, f), sensorID)
        else:
            continue
        
        # Add each data file to a list for that sensor
        if sensorID in all_data:
            all_data[sensorID].append(data)
        else:
            all_data[sensorID] = [data]

    # Concatenate each sensor's data into a single dataframe per sensor
    all_data = {id : pd.concat(all_data[id]).sort_index() for id in all_data}
    # Remove data from end of each dataset if specified.
    if isinstance(trim_ends, int):
        if trim_ends > 0:
            for id in all_data:
                all_data[id] = all_data[id].iloc[trim_ends:-trim_ends]
    elif isinstance(trim_ends, pd.DataFrame):
        for id in all_data:
            start = trim_ends.at[id, "start"]
            end = trim_ends.at[id, "end"]
            all_data[id] = all_data[id].loc[start:end]
    """
    elif isinstance(trim_ends, pd.DataFrame):
        for id in all_data:
            start_time, end_time = trim_ends.loc[id]
            if start_time is pd.NA:
                all_data[id] = all_data[id].iloc[1800:]
            else:
                all_data[id] = all_data[id].loc[start_time:]

            if end_time is pd.NA:
                all_data[id] = all_data[id].iloc[:-1800]
            else:
                all_data[id] = all_data[id].loc[:end_time]
    """

    # Concatenate all the data into a single dataframe.
    all_data = pd.concat(all_data.values(), axis=1).sort_index()
    return all_data

'''
# Cut the specified number of minutes off each end of each series of data. This
# can be used to cut off data from before / after the sensor was under water.
def strip_ends(data: pd.DataFrame, number):
    data
'''

# Plot all pressure data together
def plot_pressure(all_data: pd.DataFrame):
    all_data[[c for c in all_data.columns if c.endswith("_P")]].plot()
    pyplot.show()



# Given pressure data and corresponding offset constants for each sensor, adjust
# the pressure data according to the offset constants. The data will be adjusted
# relative to the sensor relative_to. Returns an adjusted copy of the pressure 
# data.
def adjust_pressure(pressures: pd.DataFrame, offsets: pd.DataFrame, relative_to) -> pd.DataFrame:
    if set(pressures.columns).difference(offsets.columns):
        raise ValueError("Offsets matrix is missing one or more offsets (make sure each column in pressures has a corresponding column in offsets)")
    
    if relative_to not in offsets.columns:
        raise ValueError(f"'relative_to' must be the name of a column present in both 'pressures' and 'offsets' (column given was '{relative_to}')")

    pressures = pressures.copy()
    return pressures.add(offsets[relative_to], axis=1)

# TODO: Each script used to process a batch of data should be saved with the data
# then only the functions and a template / example script should be saved on GitHub

INPATH = "/home/benc/Documents/field-data-1/"
# %% 
timestamps = pd.read_csv(
    os.path.join(INPATH, "timestamps.csv"),
    index_col=0,
    parse_dates=True
)

# %% Read all data from seperate files.

sensor_data = read_data(os.path.join(INPATH, "processed-clean"), 1800)

# %% Write all data to a single file.
#sensor_data.to_csv(os.path.join(INPATH, "processed/all.csv"))

# %% Read all data from 'processed' from a single file.
sensor_data = pd.read_csv(
    os.path.join(INPATH, "processed/all.csv"),
    index_col=0,
    parse_dates=True
)         

# %% Plot pressure data.
plot_pressure(sensor_data)
    

# %% Clean sensor data
clean_data = sensor_data
# %% Plot clean pressure data.
plot_pressure(clean_data)
# %% Write clean data to file.
clean_data.to_csv(os.path.join(INPATH, "all-clean.csv"))

# %% Read all clean data from single file
clean_data = pd.read_csv(
    os.path.join(INPATH, "all-clean.csv"),
    index_col=0,
    parse_dates=True
)         

# %% Apply sensor offsets to pressures
adjust_to = "RBR_P"
clean_columns = clean_data.columns[:]
offsets = pd.read_csv(os.path.join(INPATH, "offsets.csv"), index_col=0)

pressures = clean_data[[c for c in clean_data.columns if c.endswith("_P")]]
adjusted_pressures = adjust_pressure(pressures, offsets, adjust_to)[[c for c in clean_columns if c.endswith("_P")]]

# %% Subtract air pressure from absolute pressures.
air_pressure_transducer = "S12_P"#"DIY3-04_P"
air_pressure = adjusted_pressures[air_pressure_transducer]
absolute_pressures = adjusted_pressures.drop(columns=["DIY3-04_P", air_pressure_transducer])

water_pressures = absolute_pressures.subtract(air_pressure, axis=0)

# %% Combine water pressures back with temperature data. Ignore any temperature 
# columns whose pressure columns were dropped.
temperatures = clean_data[[c for c in clean_data.columns if c.endswith("_T") and c.replace("_T", "_P") in adjusted_pressures.columns]]
final_data = pd.concat([water_pressures, temperatures], axis=1)
# Sort columns in alphabetical order
final_data = final_data.reindex(sorted(final_data.columns), axis=1)

# %% Write data to file for each day for all sensors
if not os.path.exists(os.path.join(INPATH, "data")):
    os.mkdir(os.path.join(INPATH, "data"))

for group, df in final_data.groupby(pd.Grouper(axis=0, freq="d")):
    name = f"{datetime.strftime(group, r'%Y-%m-%d')}_data.csv"
    print(f"Saving 'data/{name}'")
    df.to_csv(os.path.join(INPATH, f"data/{name}"))


# %% 
day = pd.read_csv(
    os.path.join(INPATH, "data/2022-08-20_data.csv"),
    index_col=0,
    parse_dates=True
)

plot_pressure(day)
a=5
# %%
