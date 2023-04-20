# %%
from datetime import datetime
import os
import struct

import pandas as pd
from matplotlib import pyplot as plt


def _ms2_read_raw(raw):
    # Dictionary to store unpacked data. We convert this to a DataFrame 
    # later.
    unpacked = {"timestamp":[], "pressure":[], "temperature":[]}
    # Loop through the data by bytes. If a file has any missing bytes,
    # attempt to add NaN values to finish the row.
    try:
        for i in range(0, len(raw), 9):
            # The first 4 bytes are a Unix timestamp (unsigned int)
            timestamp = struct.unpack("<i", raw[i : i + 4])[0]
            # Convert it into a datetime timestamp object so Pandas 
            # knows what it is.
            unpacked["timestamp"].append(datetime.utcfromtimestamp(timestamp))
            # The next 4 bytes represent the pressure as a float.
            unpacked["pressure"].append(struct.unpack("<f", raw[i + 4 : i + 8])[0])
            # The last byte represents the temperature as a signed int
            unpacked["temperature"].append(struct.unpack("<B", raw[i + 8 : i + 9])[0])
            
            #print(f"Time: {timestr} \tPressure: {pressure} \tTemperature: {temperature}")
    
    except struct.error:
        print(f"Error reading {fullFileName}")
        # Add NaN values to keep the column lengths the same.
        if len(unpacked["pressure"]) < len(unpacked["timestamp"]):
            unpacked["pressure"].append(pd.NA)
        if len(unpacked["temperature"]) < len(unpacked["pressure"]):
            unpacked["temperature"].append(pd.NA)


    # Read the data into a pandas DataFraame
    return pd.DataFrame(unpacked)


def _diy3_read_raw(raw_data: str) -> pd.DataFrame:
    bufferSize = 120
    entrySize = 5
    timestampSize = 4
    # Dictionary to store unpacked data. We convert this to a DataFrame 
    # later.
    unpacked = {"timestamp":[], "pressure":[], "temperature":[]}
    # Loop through the data by bytes. If a file has any missing bytes,
    # attempt to add NaN values to finish the row.
    try:
        i = 0
        while (i < len(raw_data)):
            timestamp = struct.unpack("<i", raw_data[i : i + timestampSize])[0]
            i += timestampSize
            for j in range(i, i + bufferSize * entrySize, entrySize):
                # Convert it into a datetime timestamp object so Pandas 
                # knows what it is.
                unpacked["timestamp"].append(datetime.utcfromtimestamp(timestamp))
                # The next 4 bytes represent the pressure as a float.
                unpacked["pressure"].append(struct.unpack("<f", raw_data[j : j + 4])[0])
                # The last byte represents the temperature as a signed int
                unpacked["temperature"].append(struct.unpack("<B", raw_data[j + 4 : j + 5])[0])

                timestamp += 1
            i += bufferSize * entrySize
    except struct.error:
        print("Error reading file")
        # Add NaN values to keep the column lengths the same.
        if len(unpacked["pressure"]) < len(unpacked["timestamp"]):
            unpacked["pressure"].append(pd.NA)
        if len(unpacked["temperature"]) < len(unpacked["pressure"]):
            unpacked["temperature"].append(pd.NA)

    # Read the data into a pandas DataFraame
    return pd.DataFrame(unpacked)


def read_raw(input_dir_path: str, output_dir_path: str, start_time: datetime = None) -> pd.DataFrame:
    for full_file_name in os.listdir(input_dir_path):
        # Store the full path to the current file as a string.
        file_path = os.path.join(input_dir_path, full_file_name)
        # Get the name and extension, i.e. name.ext
        split_name, ext = os.path.splitext(full_file_name)

        # Only use .data files (there may be config or other junk files on
        # the device which we don't want to process).
        if ext != ".data":
            print(f"Ignoring file {file_path}: not a .data file)")
            continue

        # Only take data from after the specified date. If the file name isn't
        # in the right format (name_date-time.data) then skip it.
        try:
            sensor_id, data_timestamp = split_name.split("_")
        except ValueError:
            print(f"Ignoring file {file_path}: incorrect name format")
            continue
        
        # Ignore files before the start time, if specified.
        if start_time != None:
            data_timestamp = datetime.strptime(data_timestamp, r"%Y%m%d-%H%M")
            if data_timestamp < start_time:
                print(f"Ignoring file {file_path}: timestamp is earlier than start_time={start_time}")
                continue

        # Open the file as a binary file and read it.
        print("Reading " + file_path)
        with open(file_path, "rb") as file:
            raw = file.read()
        
        if sensor_id.startswith("DIY3"):
            data = _diy3_read_raw(raw)
        elif sensor_id.startswith("S"):
            data = _ms2_read_raw(raw)
        else:
            raise 
        
        # Create the output directory if it doesn't already exist.
        if not os.path.exists(output_dir_path):
            os.mkdir(output_dir_path)
        # Write the data to a CSV file with the same name as the raw 
        # data.
        data.to_csv(os.path.join(output_dir_path, f"{split_name}.csv"), index=False)


def _diy3_read_processed(path: str, sensorID: str) -> pd.DataFrame:
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
def _rbr_read_eng(path: str, sensorID: str) -> pd.DataFrame:
    # Read data and rename columns, using timestamp as the index.
    data = pd.read_table(
        filepath_or_buffer=path,
        sep="    ",
        skiprows=31, 
        usecols=[0, 1, 2], # Select Timestamp, Pressure, and Temperature
        names=["timestamp", sensorID + "_T", sensorID + "_P", ], # Rename columns
        encoding="ISO-8859-1", # Default encoding runs into some characters it can't parse
        engine="python") 
    # Convert time datetime objects
    data["timestamp"] = pd.to_datetime(data["timestamp"], format="%d-%b-%Y %H:%M:%S.000")
    data.set_index("timestamp", inplace=True)
    # Convert pressure to mbar
    data[sensorID + "_P"] *= 100 

    return data


# For reading RBR's legacy xls output.
def _rbr_read_xls(path: str, sensorID: str) -> pd.DataFrame:
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
def read_processed(input_dir_path: str) -> dict[str, pd.DataFrame]:
    all_data = {}
    for f in os.listdir(input_dir_path):
        name, ext = os.path.splitext(f)
        # Read DIY3 data
        if ext == ".csv" and "_" in name:
            sensorID, timestamp = name.split("_")
            print(f"Reading {name}")
            data = _diy3_read_processed(os.path.join(input_dir_path, f), sensorID)
        # Read RBR engineering output data
        elif ext == ".txt" and name.endswith("eng"):
            sensorID = "RBR"
            data = _rbr_read_eng(os.path.join(input_dir_path, f), sensorID)
        elif ext == ".xls":
            sensorID = "RBR"
            data = _rbr_read_xls(os.path.join(input_dir_path, f), sensorID)
        else:
            continue
        
        # Add each data file to a list for that sensor
        if sensorID in all_data:
            all_data[sensorID].append(data)
        else:
            all_data[sensorID] = [data]

    # Concatenate each sensor's data into a single dataframe per sensor
    return {id : pd.concat(all_data[id]) for id in all_data}


# Read all final processed data (one or more .csv files, each containing data 
# columns for all sensors over some time range) from a directory.
def read_final_data(dir_path: str) -> pd.DataFrame:
    all_data = []
    for f in os.listdir(dir_path):
        all_data.append(
            pd.read_csv(
                os.path.join(dir_path, f), 
                index_col=0,
                parse_dates=True
            )
        )

    data = pd.concat(all_data).sort_index()
    return data


# Plot all pressure data together and show the plot.
def plot_pressure(all_data: pd.DataFrame) -> None:
    all_data.loc[:, [c for c in all_data.columns if c.endswith("_P")]].plot()
    plt.show()
    plt.close()

# Plot all temperature data together.
def plot_temperatures(all_data: pd.DataFrame):
    all_data[[c for c in all_data.columns if c.endswith("_T")]].plot()
    plt.show()
    plt.close()

# Plot all pressure data but only show one at a time.
def plot_pressure_individually(all_data: pd.DataFrame):
    for c in all_data.columns:
        if c.endswith("_P"):
            all_data[c].plot(title=c)
            plt.show()
            plt.close()

# Given pressure data and corresponding offset constants for each sensor, adjust
# the pressure data according to the offset constants. The data will be adjusted
# relative to the sensor relative_to. Returns an adjusted copy of the pressure 
# data.
def adjust_pressure(pressures: pd.DataFrame, offsets: pd.DataFrame, relative_to: str) -> pd.DataFrame:
    if set(pressures.columns).difference(offsets.columns):
        raise ValueError("Offsets matrix is missing one or more offsets (make sure each column in pressures has a corresponding column in offsets)")
    
    if relative_to not in offsets.columns:
        raise ValueError(f"'relative_to' must be the name of a column present in both 'pressures' and 'offsets' (column given was '{relative_to}')")

    pressures = pressures.copy()
    return pressures.add(offsets[relative_to], axis=1)


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



# An example script utilizing the above functions
def main():
    # %%
    INPATH = "/home/benc/Documents/2023_april_pt_test"
    RAW_PATH = os.path.join(INPATH, "raw")
    PROCESSED_PATH = os.path.join(INPATH, "processed")
    DATA_PATH = os.path.join(INPATH, "data")

    # %% Read all raw data into csv files.
    read_raw(RAW_PATH, PROCESSED_PATH)

    # %% Read all processed data.
    # (After you have run the previous line once, subsequent runs of your script
    # can skip it and just use this line to save time when testing)
    data_dict = read_processed(PROCESSED_PATH)
    
    # %% Remove a specified number of rows of data from the start and end of 
    # each dataset.
    # trim_ends = 1800
    # for id in data_dict:
    #    data_dict[id] = data_dict[id].iloc[trim_ends:-trim_ends]
    
    
    # %% Remove data from the dataset based on start and end timestamps.
    timestamps = pd.read_csv(
        os.path.join(INPATH, "timestamps.csv"),
        index_col=0,
        parse_dates=True
    )
    for id in data_dict:
        start = timestamps.at[id, "start"]
        end = timestamps.at[id, "end"]
        data_dict[id] = data_dict[id].loc[start:end]

    # %% Concatenate all the data into a single dataframe.
    sensor_data = pd.concat(data_dict.values(), axis=1, copy=False).sort_index()     

    # %% Plot pressure data.
    plot_pressure(sensor_data)        

    # %% Clean sensor data by removing any observed / known outliers. In this 
    # example we don't change anything.
    clean_data = sensor_data
    # %% Plot clean pressure data.
    plot_pressure(clean_data)
    # %% Write clean data to file.
    clean_data.to_csv(os.path.join(INPATH, "all-clean.csv"))
    
    # %% Read all clean data from single file
    # (Once you've generated your clean data and you're happy with it, you can
    # skip the previous lines in your script and just start from this one to 
    # save time when testing your script)
    clean_data = pd.read_csv(
        os.path.join(INPATH, "all-clean.csv"),
        index_col=0,
        parse_dates=True
    )       

    # %% Determine sensor offsets and calculate missing values when possible.
    # (Only the first statement (pd.read(...)) is needed if the offsets matrix
    # has already been filled in previously) 
    offsets = pd.read_csv(
        os.path.join(INPATH, "offsets.csv"),
        index_col=0
    )
    offsets = infer_offsets(offsets)
    offsets.to_csv(os.path.join(INPATH, "offsets_adj.csv"))

    # %% Apply sensor offsets to pressures
    adjust_to = "RBR_P"
    clean_columns = clean_data.columns[:]  # Make a copy for later

    pressures = clean_data[[c for c in clean_data.columns if c.endswith("_P")]]
    adjusted_pressures = adjust_pressure(pressures, offsets, adjust_to)[[c for c in clean_columns if c.endswith("_P")]]

    # %% Subtract air pressure from absolute pressures.
    air_pressure_transducer = "DIY3-04_P"
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
    if not os.path.exists(DATA_PATH):
        os.mkdir(DATA_PATH)

    for group, df in final_data.groupby(pd.Grouper(axis=0, freq="d")):
        name = f"{datetime.strftime(group, r'%Y-%m-%d')}_data.csv"
        print(f"Saving 'data/{name}'")
        df.to_csv(os.path.join(DATA_PATH, "name"))

    
# %%
if __name__ == "__main__":
    main() 