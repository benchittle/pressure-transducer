import os

import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

INPUT_PATH = r"E:\OneDrive - University of Windsor\Pressure Transducer Work\Pressure Transducer Tests" + "\\"
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
            if len(sensor) == 1:
                sensor = "0" + sensor
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
    DATES = ["2021-08-25", "2021-08-25", "2021-08-26", "2021-08-26", "2021-08-26", "2021-08-30", "2021-08-30", "2021-08-30", "2021-08-30"]
    START_TIMES = [["16:20", "16:30", "16:48"], ["17:05", "17:21", "18:31"], ["17:01", "17:09", "17:20"], ["17:35", "17:53", "18:29"], ["18:40", "18:48", "18:58"], ["12:17", "12:50", "13:24"], ["13:33", "13:52", "14:29"], ["14:48", "15:46", "16:50"], ["17:09", "17:21", "17:44"]]
    END_TIMES = [["16:26", "16:35", "16:52"], ["17:15", "17:54", "18:35"], ["17:05", "17:14", "17:27"], ["17:40", "17:58", "18:35"], ["18:44", "18:53", "19:03"], ["12:23", "12:56", "13:29"], ["13:43", "14:20", "14:44"], ["14:52", "16:00", "17:07"], ["17:14", "17:40", "17:50"]]
    folders = [f + "\\" for f in os.listdir(INPUT_PATH) if f.startswith("calibration")]
    sensor_stats = pd.DataFrame()
    all_data = []

    for folder, d, s, e in zip(folders, DATES, START_TIMES, END_TIMES):
        start_times = [d + " " + time for time in s]
        end_times = [d + " " + time for time in e]    

        #data = pd.DataFrame()

        test_num = folder.split("_")[1]

        sensors_raw = read_ms2_csvs(INPUT_PATH + folder).add_suffix("_" + test_num)
        rbr_raw = read_rbr_xls(INPUT_PATH + folder).add_suffix("_" + test_num)

        #residuals = []
        print(folder)
        for i, (start, end) in enumerate(zip(start_times, end_times)):
            sensor_data = sensors_raw.loc[start:end].iloc[60:-60]
            rbr_data = rbr_raw.loc[start:end].iloc[60:-60]
            res = sensor_data.subtract(rbr_data, axis=0)
           # residuals.append(res.add_suffix("_" + str(i)))
            sensor_stats.loc["Offset {}".format(i), sensors_raw.columns] = res.mean()

#        residuals = pd.concat(residuals, axis=1)
 #       residuals = residuals.reindex(sorted(residuals.columns), axis=1)
    
    sensor_stats.to_csv(OUTPUT_PATH)

    print("Done")
    return sensor_stats

if __name__ == "__main__":
    sensor_stats = main()