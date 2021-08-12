import os

import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

INPUT_PATH = r"C:\Users\Uni\Desktop\Ben\test" + "\\"
OUTPUT_PATH = "output.xlsx"

def read_ms2_csvs(INPUT_PATH):
    
    csvs = []
    for file_name in os.listdir(INPUT_PATH):
        head, extension = os.path.splitext(file_name)
        # Valid name example: S1_20210813-1345.csv
        if extension == ".csv":
            sensor, timestamp = head.split("_")
            data = pd.read_csv(
                filepath_or_buffer=INPUT_PATH + file_name, 
                skiprows=2, 
                usecols=["datetime", "pressure"], 
                parse_dates=["datetime"], 
                dayfirst=True)
            data.drop_duplicates("datetime", inplace=True)
            data.set_index("datetime", inplace=True)
            data.rename(columns={"pressure" : sensor}, inplace=True)
            
            csvs.append(data.squeeze())
    
    return pd.concat(csvs, axis=1)



def read_rbr_xls(INPUT_PATH):
    for file_name in os.listdir(INPUT_PATH):
        if file_name.endswith(".xls"):
            data = pd.read_excel(
                io=INPUT_PATH + file_name,
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


def plot_compare(sensor_data):
    fig, ax = plt.subplots()
    x = sensor_data.index
    for s in sensor_data:
        ax.plot(x, sensor_data[s], "-")

    ax.set_xlabel("Time")
    ax.set_ylabel("Pressure (mbar)")
    ax.legend()


def plot_against(rbr_data, sensor_data):
    fig, ax = plt.subplots()
    ax.plot(sensor_data, rbr_data, "b.")

    ax.set_xlabel(sensor_data.name)
    ax.set_ylabel("rbr")


def get_valid_int(prompt):
    while True: 
        try:
            num = int(input(prompt))
            return num
        except ValueError:
            print("INVALID INPUT: Enter a valid integer")
        


def main():
    start_time = "2021-08-10 20:21:30"
    end_time = "2021-08-10 20:36:50"

    # Read RBR pressure data.
    rbr_raw = read_rbr_xls(INPUT_PATH)
    # Read all collected pressure data from sensors.
    sensors_raw = read_ms2_csvs(INPUT_PATH)

    # DataFrame for storing aggreate statistics about sensor performance.
    sensor_stats = pd.DataFrame(columns=sensors_raw.columns)

    # Get the data from during the testing period.
    rbr = rbr_raw.loc[start_time:end_time]
    sensors = sensors_raw.loc[start_time:end_time].interpolate(limit=2)

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

    print("\n\nSTATS")
    print(sensor_stats)
    print("\n")

    with pd.ExcelWriter(OUTPUT_PATH) as writer:
        pd.concat([rbr_raw, sensors_raw], axis=1).to_excel(writer, "Raw")
        pd.concat([rbr, sensors], axis=1).to_excel(writer, "Time Filtered")
        pd.concat([rbr, sensors_corrected], axis=1).to_excel(writer, "Corrected")
        sensor_stats.to_excel(writer, "Stats")

    menu_string = ("Plotting:\n"
            "1. Plot the pressure curves of all sensors\n"
            "2. Plot the pressure curves of all sensors within the specified time range\n"
            "3. Plot a sensor's corrected pressure (x axis) against the RBR (y axis)\n"
            "4. Show plots\n"
            "5. Quit\n")    
    
    while True:
        action = get_valid_int(menu_string)
            
        if action == 1:
            plot_compare(pd.concat([rbr_raw, sensors_raw], axis=1))
        elif action == 2:
            plot_compare(pd.concat([rbr, sensors], axis=1))
        elif action == 3:
            while True:
                try:
                    plot_against(rbr, sensors_corrected["S" + str(get_valid_int())] )
                    break
                except IndexError:
                    print("INVALID INPUT: There is no data for that sensor")
        elif action == 4:
            print("Showing plots\n")
            plt.show()
        elif action == 5:
            print("Quitting")
            break
        else:
            print("INVALID INPUT: Enter an integer between 1 and 4")
        

if __name__ == "__main__":
    main()