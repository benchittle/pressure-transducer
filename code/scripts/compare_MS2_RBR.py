import os

import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

PATH = "test_data\\"

def read_ms2_csvs(path):
    
    csvs = []
    for file_name in os.listdir(path):
        head, extension = os.path.splitext(file_name)
        # Valid name example: S1_20210813-1345.csv
        if extension == ".csv":
            sensor, timestamp = head.split("_")
            data = pd.read_csv(
                filepath_or_buffer=path + file_name, 
                skiprows=2, 
                usecols=["datetime", "pressure"], 
                parse_dates=["datetime"], 
                dayfirst=True)
            data.set_index("datetime", inplace=True)
            data.rename(columns={"pressure" : sensor}, inplace=True)
            
            csvs.append(data.squeeze())
    
    return pd.concat(csvs, axis=1)



def read_rbr_xls(path):
    for file_name in os.listdir(path):
        if file_name.endswith(".xls"):
            data = pd.read_excel(
                io=path + file_name,
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
    start_time = "2021-08-08 17:16:00"
    end_time = "2021-08-08 17:29:00"

    # Read RBR pressure data.
    rbr_raw = read_rbr_xls(PATH)
    # Read all collected pressure data from sensors.
    sensors_raw = read_ms2_csvs(PATH)

    # DataFrame for storing aggreate statistics about sensor performance.
    sensor_stats = pd.DataFrame(columns=sensors_raw.columns)

    # Get the data from during the testing period.
    rbr = rbr_raw.loc[start_time:end_time]
    sensors = sensors_raw.loc[start_time:end_time]

    # Calculate the residuals for each sensors (MS2 - RBR).
    residuals = sensors.subtract(rbr, axis=0)
    # Calculate the sum of residuals for each sensor. The closer the value is
    # to 0, the closer the sensor is to the RBR.
    sensor_stats.loc["sum of residuals"] = residuals.sum()
    # Calculate the mean absolute error for each sensor.
    sensor_stats.loc["mean abs error"] = residuals.abs().mean()
    # Perform a linear regression of each MS2 (x) against the RBR (y) and 
    # record the coefficients.
    for s in sensors:
        slope, intercept = np.polyfit(x=sensors[s], y=rbr, deg=1)
        sensor_stats.at["slope", s] = slope
        sensor_stats.at["intercept", s] = intercept

    # Caculate corrected pressures for each sensor based on the previously 
    # calculated coefficients.
    sensors_corrected = sensors * sensor_stats.loc["slope"] + sensor_stats.loc["intercept"]
    # Calculate the mean absolute error for the corrected values for each 
    # sensor.
    sensor_stats.loc["corrected mean abs error"] = sensors_corrected.subtract(rbr, axis=0).abs().mean()

    # Calculate the coefficient of determination for each of the corrected
    # sensors against the RBR.
    sensor_stats.loc["r squared"] = sensors_corrected.corrwith(rbr) ** 2

    return

if __name__ == "__main__":
    main()