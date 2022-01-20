import os

import pandas as pd
import numpy as np

DAY = 4

INPUT_PATH = r"E:\OneDrive - University of Windsor\Pressure Transducer Work\Pressure Transducer Tests\peche_day_{}".format(DAY) + "\\"
OFFSET_INPUT_PATH = r"E:\OneDrive - University of Windsor\Pressure Transducer Work\Pressure Transducer Tests\offsets.csv"
OUTPUT_PATH = r"E:\OneDrive - University of Windsor\Pressure Transducer Work\Pressure Transducer Tests\peche_day_{0}\peche_day_{0}.xlsx".format(DAY)

DAY1_TRANSECTS = []
DAY2_TRANSECTS = [
    "S07", "S08", "S09", "S10", "S06",
    "S12", "S13", "S16", "S15", "S14",  
    "S02", "S04", "S20", "S03", "S05", 
    "S01"]
DAY3_TRANSECTS = [
    "S07", "S08", "S09", "S10", "S06",
    "S12", "S13", "S16", "S15", "S14",  
    "S17", "S04", "S18", "S03", "S05", 
    "S01"]

DAY4_TRANSECTS = [
    "S07", "S08", "S09", "S10", "S06",
    "S12", "S13", "S16", "S15", "S14",  
    "S17", "S04", "S18", "S03", "S05", 
    "S01"]

def read_ms2_csvs(input_path):
    csvs = []
    # Read each csv in the given directory as sensor data.
    for file_name in os.listdir(input_path):
        head, extension = os.path.splitext(file_name)
        # Valid name example: S1_20210813-1345.csv
        if extension == ".csv":
            sensor, timestamp = head.split("_")
            if len(sensor) == 2:
                sensor = "S0" + sensor[-1]
            # Read the datetime and pressure columns.
            data = pd.read_csv(
                filepath_or_buffer=input_path + file_name, 
                skiprows=2, 
                usecols=["datetime", "pressure"], 
                parse_dates=["datetime"], 
                dayfirst=True)
            # Every so often two readings occur in the same second. It's easier
            # to work with the data if we drop these.
            # ISSUE: THIS POSSIBLY DROPS BOTH RATHER THAN JUST ONE OF THE DUPLICATES
            #data.drop_duplicates("datetime", inplace=True)
            
            data.set_index("datetime", inplace=True)
            # Rename the data's pressure column to the name of the sensor that 
            # produced it.
            data.rename(columns={"pressure" : sensor}, inplace=True)
            
            csvs.append(data.squeeze())
    
    return pd.concat(csvs, axis=1)

def main():
    # Read sensor data into a single data frame with DateTimes in the index
    # and pressure reading for each sensor as the columns.
    sensor_data = read_ms2_csvs(INPUT_PATH)
    # Reorder the columns based on the order of the transects.
    sensor_data = sensor_data.reindex(DAY3_TRANSECTS, axis=1)
    
    # Read in offset constants to adjust data.
    offsets = pd.read_csv(OFFSET_INPUT_PATH, usecols=["Sensor", "Offset"])
    # Squeeze to series and drop offset values for sensors that are not present
    # in the data.
    offsets = offsets.set_index("Sensor").squeeze()[sensor_data.columns]

    # Write data to Excel:
    with pd.ExcelWriter(OUTPUT_PATH) as writer:
        # Raw data
        sensor_data.to_excel(writer, "raw")
        # Adjusted data (adding the corresponding offset to each pressure 
        # reading)
        (sensor_data + offsets).to_excel(writer, "adjusted")
        # Offset values used.
        offsets.to_excel(writer, "offsets")

    print("Done")

if __name__ == "__main__":
    main()