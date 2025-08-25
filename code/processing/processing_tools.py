"""
This file contains functions for processing your transducer's data using Pandas.

You can import and use it in your own scripts for custom/advanced processing, or
run it from the command line for some quick data visualization.

Run this file with the -h flag for help
"""


import argparse
from datetime import datetime, timedelta, timezone
import glob
import os
import pickle
import struct

import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

import time

HEADER_VERSION_STRING_SIZE = 4
PROFILE = False

def read_binary_to_dataframe(input_file: str) -> dict:
    if PROFILE:
        start = time.perf_counter()
    
    with open(input_file, "rb") as bin_file:
        header_version_string = bin_file.read(HEADER_VERSION_STRING_SIZE)
        match header_version_string.decode():
            case "V000":
                # Remaining header size
                REMAINING_HEADER_SIZE = 19
                header_data = bin_file.read(REMAINING_HEADER_SIZE)

                # Data format: 
                # 16 byte device name (null terminated)
                # 1 byte sampling frequency
                # 2 byte chunk size
                header_format = "<16sBH"
                header_unpacked: tuple[bytes, int, int] = struct.unpack(header_format, header_data)

                header = {
                    # Remove null bytes from device name string
                    "device_name" : header_unpacked[0].decode().replace("\x00", ""),
                    "sample_frequency" : header_unpacked[1],
                    "samples_per_chunk" : header_unpacked[2]
                }

                if PROFILE:
                    print(start - time.perf_counter())
                    start = time.perf_counter()
                    print("np reading")

                # We'll use numpy to read our custom binary data format
                dt = np.dtype({
                    "names": ["timestamp", "temperature", "pressure"],
                    "formats": ["<i4", "<f4", f"<{header["samples_per_chunk"]}f4"]
                })
                np_data = np.fromfile(bin_file, dt)

                # The last chunk of data might contain fewer samples (if the 
                # user pressed the shutdown button for example). This case has 
                # to be handled manually.
                if remaining_data := bin_file.read():
                    # Determine number of samples in the last chunk
                    remaining_samples = (len(remaining_data) - 4 - 4) / 4
                    if remaining_samples != int(remaining_samples):
                        raise ValueError(f"Failed to read {input_file}: incorrect binary format")
                    remaining_samples = int(remaining_samples)

                    # Read the data just like above. We'll combine it with the 
                    # other data later.
                    dt = np.dtype({
                        "names": ["timestamp", "temperature", "pressure"],
                        "formats": ["<i4", "<f4", f"<{remaining_samples}f4"]
                    })
                    np_remaining = np.frombuffer(remaining_data, dt)

                if PROFILE:
                    print(start - time.perf_counter())
                    start = time.perf_counter()
                    print("making df")

                # I haven't figured out a nice way of wrangling the data into 
                # the format I'm looking for, so there are quite a few steps.
                # First we'll read the data into a dataframe.
                df = pd.DataFrame(
                    [np_data["timestamp"], np_data["temperature"], np_data["pressure"]]
                ) 

                # For some reason, pandas likes to read the data as rows 
                # instead of columns, so we'll transpose it.
                df = df.T

                # Append the last chunk of data if there was one
                if remaining_data:
                    remaining_df = pd.DataFrame(
                        [np_remaining["timestamp"], np_remaining["temperature"], np_remaining["pressure"]]
                    )
                    df = pd.concat([df, remaining_df.T])

                if PROFILE:
                    print(start - time.perf_counter())
                    start = time.perf_counter()
                    print("renaming")

                # Now we can give our columns names.
                df.rename(
                    columns={
                        0 : "timestamp",
                        1 : "temperature",
                        2 : "pressure"
                    },
                    inplace=True
                )

                if PROFILE:
                    print(start - time.perf_counter())
                    start = time.perf_counter()
                    print("interpreting timestamps")

                # Interpret the timestamp column as datetimes.
                df["timestamp"] = pd.to_datetime(df["timestamp"], unit="s")
                
                if PROFILE:
                    print(start - time.perf_counter())
                    start = time.perf_counter()
                    print("explode")
                
                # Split up the data since temperature values are not as frequent 
                # as pressure.
                temperature = df["temperature"].set_axis(df["timestamp"])
                df.drop(columns=["temperature"], inplace=True)

                # Flatten the pressure column. This will copy the timestamp 
                # values in each new row for each chunk of pressure values. 
                df = df.explode("pressure")

                if PROFILE:
                    print(start - time.perf_counter())
                    start = time.perf_counter()
                    print("convert dtpes")

                # Convert from "object" types to numeric types.
                df = df.astype({"pressure" : "float32"})
                temperature = temperature.astype("float32")

                if PROFILE:
                    print(start - time.perf_counter())
                    start = time.perf_counter()
                    print("inc timesatmps")
                
                # Increment timestamps according to the sample frequency.
                df["timestamp"] += pd.to_timedelta(df.groupby("timestamp").cumcount() * (1000 / header["sample_frequency"]), unit="ms")

                if PROFILE:
                    print(start - time.perf_counter())
                    start = time.perf_counter()
                    print("set index")

                # Set the index for pressure values
                df.set_index("timestamp", inplace=True)
                pressure = df

                return {
                    "header" : header,
                    "pressure" : pressure,
                    "temperature" : temperature
                }
            case _:
                raise ValueError(f"Input data file has an unknown header version string: {header_version_string}")
            

def read_data(paths: str | list[str]) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Read raw .data files into two sorted dataframes, one for pressure and one 
    for temperature.

    `paths` can be one or more paths to folders or .data files. Folders will be
    globbed for any file ending in ".data" (not recursive).
    """
    all_temperature = {}
    all_pressure = {}

    if isinstance(paths, str):
        paths = [paths]

    for path in paths:
        if os.path.isdir(path):
            for file in glob.glob(os.path.join(path, "*.data")): 
                print(f"Reading {file}")
                data = read_binary_to_dataframe(file)
                
                all_temperature.setdefault(data["header"]["device_name"], []).append(data["temperature"])
                all_pressure.setdefault(data["header"]["device_name"], []).append(data["pressure"])
        else:
            data = read_binary_to_dataframe(path)
            all_temperature.setdefault(data["header"]["device_name"], []).append(data["temperature"])
            all_pressure.setdefault(data["header"]["device_name"], []).append(data["pressure"])

    pressure = pd.concat([pd.concat(device_dfs) for device_dfs in all_pressure.values()], axis="columns", sort=True, keys=all_pressure.keys())
    pressure.sort_index(axis="columns", inplace=True)
    pressure = pressure.droplevel(1, axis="columns")

    temperature = pd.concat([pd.concat(device_dfs) for device_dfs in all_temperature.values()], axis="columns", sort=True, keys=all_temperature.keys())
    temperature.sort_index(axis="columns", inplace=True)

    return (pressure, temperature)


def main():
    # ==== Argument Parsing ====
    parser = argparse.ArgumentParser(description="Example script to process and visualize raw data. Run the subcommands below with the -h flag for additional help")
    subparsers = parser.add_subparsers(dest="command", required=True)

   # read command
    read_parser = subparsers.add_parser("read", help="Read data from .data files into a pickle file for further processing. Must be done before other commands can be used.")
    read_parser.add_argument(
        "input",
        nargs="+",
        help="list of .data files or folders to search for .data files"
    )
    read_parser.add_argument(
        "-o", "--output",
        required=True,
        help="output path for .pkl file"
    )

    # summarize command
    summarize_parser = subparsers.add_parser("summarize", help="Summarize data from pickle file")
    summarize_parser.add_argument(
        "input",
        help=".pkl file for your data"
    )

    # plot command
    plot_parser = subparsers.add_parser("plot", help="Plot data from pickle file")
    plot_parser.add_argument(
        "input",
        help=".pkl file for your data"
    )
    plot_exclusive_parser = plot_parser.add_mutually_exclusive_group()
    plot_exclusive_parser.add_argument(
        "-i", "--include",
        action="append",
        metavar="NAME",
        help="only data for the given device name will be plotted (can be used more than once)"
    )
    plot_exclusive_parser.add_argument(
        "-e", "--exclude",
        action="append",
        metavar="NAME",
        help="data for the given device name will be excluded (can be used more than once)"
    )
    plot_parser.add_argument(
        "--start",
        metavar="TIMESTAMP",
        help="only show data after this timestamp (YYYY-MM-DD hh:mm:ss)"
    )
    plot_parser.add_argument(
        "--end",
        metavar="TIMESTAMP",
        help="only show data up to this timestamp (YYYY-MM-DD hh:mm:ss)"
    )
    plot_parser.add_argument(
        "--kind",
        choices=["line", "box"],
        default="line",
        help="type of plot (default: line)"
    )
    plot_parser.add_argument(
        "--remove-outliers",
        action="store_true",
        help="remove and interpolate outliers in pressure data"
    )
    plot_parser.add_argument(
        "--resample",
        metavar="PERIOD",
        help="resample pressure data (argument will be passed directly to DataFrame.resample, so '10s', '1min', '0.5s' are all valid inputs (without quotes))"
    )

    # Parse arguments
    args = parser.parse_args()


    # ==== Data Processing ====
    if args.command == "read":
        print("Reading data...")
        data = read_data(args.input)
        
        output_path: str = args.output
        if not output_path.endswith(".pkl"):
            output_path = output_path + ".pkl"

        print(f"Writing data to {args.output}")
        with open(args.output, "wb") as f:
            pickle.dump(data, f)
        
        print("Done")

    elif args.command == "summarize":
        # Read previously unpacked data
        with open(args.input, "rb") as f:
            pressure: pd.DataFrame
            temperature: pd.DataFrame
            pressure, temperature = pickle.load(f)
        
        print("\n== Pressure Data Summary (Units: mbar) ==")
        pressure_stats = {
            "missed_samples" : {}, 
            "first_sample": {}, 
            "last_sample": {}
        }
        for c in pressure.columns:
            pressure_stats["first_sample"][c] = pressure[c].first_valid_index()
            pressure_stats["last_sample"][c] = pressure[c].last_valid_index()
            valid_data = pressure[c].dropna()

            # Count missed samples. Could also add an option to print their 
            # timestamps
            if valid_data.index.inferred_freq is None:
                target_freq = valid_data.index[1] - valid_data.index[0]
                pressure_stats["missed_samples"][c] = valid_data.asfreq(target_freq).isna().sum()
            else:
                pressure_stats["missed_samples"][c] = 0       
        
        print(pd.concat([pressure.describe(), pd.DataFrame(pressure_stats).T]))
        
        print("\n== Temperature Data Summary (Units: degrees C) ==")
        print(temperature.describe())

    elif args.command == "plot":
        # Read previously unpacked data
        with open(args.input, "rb") as f:
            pressure: pd.DataFrame
            temperature: pd.DataFrame
            pressure, temperature = pickle.load(f)

        if args.include:
            pressure = pressure[args.include]
            temperature = temperature[args.include]
        elif args.exclude:
            pressure.drop(columns=args.exclude, inplace=True)
            temperature.drop(columns=args.exclude, inplace=True)

        if args.start:
            pressure = pressure.loc[args.start:]
            temperature = temperature.loc[args.start:]
        if args.end:
            pressure = pressure.loc[:args.end]
            temperature = temperature.loc[:args.end]

        if args.remove_outliers:
            pressure = pressure.where((pressure < pressure.mean() + 3 * pressure.std()) & (pressure > pressure.mean() - 3 * pressure.std())).interpolate()

        if args.resample:
            pressure = pressure.resample(args.resample).mean()

        pressure.plot(kind=args.kind)
        if args.kind == "line":
            temperature.interpolate().plot(kind=args.kind)
        else:
            temperature.plot(kind=args.kind)

        plt.show()


if __name__ == "__main__":
   main()