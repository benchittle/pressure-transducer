import struct
from datetime import datetime
import pandas as pd
import os

# Path to media mount folder.
INPATH = "/home/benc/Documents/DIY4_out/first_test/" #"/run/media/benc/" ##

# Data files from before this data will be ignored.
START = datetime(2022, 11, 3, 12, 35)

# TODO: Make faster read function, maybe do it in C

# Used to read 
def readV1(raw):
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


def readV2(raw):
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
        while (i < len(raw)):
            timestamp = struct.unpack("<i", raw[i : i + timestampSize])[0]
            i += timestampSize
            for j in range(i, i + bufferSize * entrySize, entrySize):
                # Convert it into a datetime timestamp object so Pandas 
                # knows what it is.
                unpacked["timestamp"].append(datetime.utcfromtimestamp(timestamp))
                # The next 4 bytes represent the pressure as a float.
                unpacked["pressure"].append(struct.unpack("<f", raw[j : j + 4])[0])
                # The last byte represents the temperature as a signed int
                unpacked["temperature"].append(struct.unpack("<B", raw[j + 4 : j + 5])[0])

                timestamp += 1
            i += bufferSize * entrySize
                
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


# Loop through all external devices (TODO: Only loop through sensor cards)
#for folder in os.listdir(INPATH):
folder = "raw"
    # Loop through all files in the root folder of the device.
for fullFileName in os.listdir(INPATH + folder + "/"):
    # Store the full path to the current file as a string.
    filePath = f"{INPATH}{folder}/{fullFileName}"
    # Get the name and extension, i.e. name.ext
    splitName, ext = os.path.splitext(fullFileName)

    # Only use .data files (there may be config or other junk files on
    # the device which we don't want to process).
    if ext != ".data":
        continue

    # Only take data from after the specified date. If the file name isn't
    # in the right format (name_date-time.data) then skip it.
    try:
        sensorID, startTime = splitName.split("_")
        startTime = datetime.strptime(startTime, r"%Y%m%d-%H%M")
        if startTime < START:
            continue
    except ValueError:
        print(f"Skipping {filePath}: incorrect name format")
        continue

    # Open the file as a binary file and read it.
    print("Reading " + filePath)
    with open(filePath, "rb") as file:
        raw = file.read()

    if splitName.startswith("DIY"):
        data = readV2(raw)
    else:
        data = readV1(raw)
    
    # Create the output directory if it doesn't already exist.
    if not os.path.exists(os.path.join(INPATH, "processed")):
        os.mkdir(os.path.join(INPATH, "processed"))
    # Write the data to a CSV file with the same name as the raw 
    # data.
    data.to_csv(os.path.join(INPATH, f"processed/{splitName}.csv"), index=False)