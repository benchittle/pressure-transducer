import os

import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

INPUT_PATH = r"C:\Users\Uni\Desktop\Ben\calibration_test_data\calibration_0_data_s2s6s7s18" + "\\"
OUTPUT_PATH = "output.xlsx"

WATER_DENSITY = 998.02 # Kg/m^3
GRAVITY = 9.806 # N/Kg

def read_ms2_csvs(input_path):
    csvs = []
    for file_name in os.listdir(input_path):
        head, extension = os.path.splitext(file_name)
        # Valid name example: S1_20210813-1345.csv
        if extension == ".csv":
            sensor, timestamp = head.split("_")
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



def read_rbr_xls(input_path):
    for file_name in os.listdir(input_path):
        if file_name.endswith(".xls"):
            data = pd.read_excel(
                io=input_path + file_name,
                skiprows=5, 
                usecols=[0, 2], # Select Timestamp and Pressure columns
                names=["datetime", "pressure"]) # Rename columns
            # Convert time datetime objects
            data["datetime"] = pd.to_datetime(data["datetime"], dayfirst=True)
            data.set_index("datetime", inplace=True)
            # Convert pressure to mbar
            data["pressure"] *= 100 
            data.rename(columns={"pressure" : "rbr"}, inplace=True)
            
    return data.squeeze()


def main():
    
    
    date = "2021-08-25 "
    start_times = ["16:20:00", "16:30:00", "16:48:00"]
    end_times = ["16:26:00", "16:35:00", "16:52:00"]

    start_times = [date + time for time in start_times]
    end_times = [date + time for time in end_times]    

    sensors_raw = read_ms2_csvs(INPUT_PATH)
    rbr_raw = read_rbr_xls(INPUT_PATH)

    sensor_stats = pd.DataFrame(columns=sensors_raw.columns)
    residuals = []

    for i, (start, end) in enumerate(zip(start_times, end_times)):
        sensor_data = sensors_raw.loc[start:end].iloc[60:-60]
        rbr_data = rbr_raw.loc[start:end].iloc[60:-60]
        res = sensor_data.subtract(rbr_data, axis=0)
        residuals.append(res.add_suffix("_" + str(i)))
        sensor_stats.loc["Offset {}".format(i)] = res.mean()

    residuals = pd.concat(residuals, axis=1)
    residuals = residuals.reindex(sorted(residuals.columns), axis=1)
    
    print("Done")

if __name__ == "__main__":
    main()