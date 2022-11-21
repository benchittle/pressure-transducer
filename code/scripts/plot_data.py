# %%
import pandas as pd
from datetime import datetime
import os
from matplotlib import pyplot

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
def read_data(dir_path) -> pd.DataFrame:
    all_data = {}
    for f in os.listdir(dir_path):
        name, ext = os.path.splitext(f)
        # Read DIY3 data
        if ext == ".csv" and "_" in name:
            sensorID, timestamp = name.split("_")
            print(f"Reading {name}")
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
    all_data = {id : pd.concat(all_data[id]) for id in all_data}

    # Concatenate all the data into a single dataframe.
    all_data = pd.concat(all_data.values(), axis=1).sort_index()
    # Sort the columns to be in alphabetical order.
    all_data = all_data.reindex(sorted(all_data.columns), axis=1)

    return all_data


# Plot all pressure data together
def plot_pressure(all_data: pd.DataFrame):
    all_data[[c for c in all_data.columns if c.endswith("_P")]].plot()
    pyplot.show()


# Plot all pressure data but only show one at a time.
def plot_pressure_individually(all_data: pd.DataFrame):
    for c in all_data.columns:
        if c.endswith("_P"):
            all_data[c].plot(title=c)
            pyplot.show()


# Plot all temperature data together.
def plot_temperature(all_data: pd.DataFrame):
    all_data[[c for c in all_data.columns if c.endswith("_T")]].plot()
    pyplot.show()


# Given a matrix of offset constants, insert any additional constants that can 
# can be inferred and return a new matrix.
def infer_offsets(offsets: pd.DataFrame) -> pd.DataFrame:
    offsets = offsets.copy()
    for row in range(len(offsets.columns)):
        for col in range(len(offsets.columns)):
            for i in range(len(offsets.columns)):
                coef = offsets.iat[row, col]
                row_coef = offsets.iat[row, i]
                col_coef = offsets.iat[i, col]
                if pd.isna(coef):
                    if not pd.isna(row_coef) and not pd.isna(col_coef):        
                        offsets.iat[row, col] = row_coef + col_coef
                elif pd.isna(row_coef):
                    if not pd.isna(col_coef):
                        offsets.iat[row, i] = coef - col_coef
                elif pd.isna(col_coef):
                    offsets.iat[i, col] = coef - row_coef
    return offsets




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



### The following cells are useful for determining sensor offsets from test data
# %% Read and plot data.
INPATH = "/home/benc/Documents/field-work-4/processed/"
sensor_data = read_data(INPATH)

# %%
sensor_data["S12_P"].plot()
pyplot.show()
#plot_pressure_individually(sensor_data)

a=1
exit()

# %% Separate control data and test data.
pressures = sensor_data[[c for c in sensor_data.columns if c.endswith("_P")]]
control_sensor = pressures["DIY3-07_P"]
test_sensors = pressures.drop(columns=["DIY3-07_P"])

# %% Plot differences (offsets) from control sensor for each test sensor and print correlation values
diffs = test_sensors.subtract(control_sensor, axis=0)
print(test_sensors.corrwith(control_sensor))
diffs.plot()

# %% Print the average offset for each test sensor using the specified time interval.
start_time = "2022-05-29 18:00:00"
end_time = "2022-05-30 3:00:00"

offsets = diffs[start_time:end_time].mean()
print(offsets)

# %% Read offsets file and interpolate in missing values
INPATH = "/home/benc/Documents/field-work-2/"
offsets = pd.read_csv(
    os.path.join(INPATH, "offsets.csv"),
    index_col=0
)
offsets = infer_offsets(offsets)
offsets.to_csv(os.path.join(INPATH, "offsets_adj.csv"))
### 
    
    #print(adjusted)
    #plot_pressure(adjusted[["RBR_P"]].dropna())
    #abs_pressures = adjusted[[c for c in adjusted.columns if not c.startswith("S12")]]
    #air = adjusted["S12_P"]
    #ater = abs_pressures.subtract(air, axis=0)
    #water.plot()
    #sensor_data[["S12_P", 'DIY3-04_P']].plot()
    #pyplot.show()
    #pressures = sensor_data[[c for c in sensor_data.columns if c.endswith("_P") and not c.startswith("DIY3-10")]]
    #pressures.subtract(pressures["DIY3-02_P"], axis=0).plot()
    #pyplot.show()
    #plot_pressure(sensor_data[[c for c in sensor_data if not c.startswith("DIY3-10")]])
    #sensor_data.to_csv(os.path.join(INPATH, "all.csv"))
# %%
