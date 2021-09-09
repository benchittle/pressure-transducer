import os

import pandas as pd
import numpy as np

DAY = 3

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

def read_ms2_csvs(input_path):
    csvs = []
    for file_name in os.listdir(input_path):
        head, extension = os.path.splitext(file_name)
        # Valid name example: S1_20210813-1345.csv
        if extension == ".csv":
            sensor, timestamp = head.split("_")
            if len(sensor) == 2:
                sensor = "S0" + sensor[-1]
            data = pd.read_csv(
                filepath_or_buffer=input_path + file_name, 
                skiprows=2, 
                usecols=["datetime", "pressure"], 
                parse_dates=["datetime"], 
                dayfirst=True)
            data.drop_duplicates("datetime", inplace=True)
            data.set_index("datetime", inplace=True)
            data.rename(columns={"pressure" : sensor}, inplace=True)
            
            csvs.append(data.squeeze())
    
    return pd.concat(csvs, axis=1)

def main():
    sensor_data = read_ms2_csvs(INPUT_PATH)
    sensor_data = sensor_data.reindex(DAY3_TRANSECTS, axis=1)
    
    offsets = pd.read_csv(OFFSET_INPUT_PATH, usecols=["Sensor", "Offset"])
    offsets = offsets.set_index("Sensor").squeeze()[sensor_data.columns]

    with pd.ExcelWriter(OUTPUT_PATH) as writer:
        sensor_data.to_excel(writer, "raw")
        (sensor_data + offsets).to_excel(writer, "adjusted")
        offsets.to_excel(writer, "offsets")

    print("Done")

if __name__ == "__main__":
    main()