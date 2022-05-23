import struct
from datetime import datetime
import pandas as pd
import os

# Path to media mount folder.
INPATH = "/run/media/benc/"
# Path to output folder. A subfolder will be created for each card that is 
# processed.
OUTPATH = "/home/benc/Documents/DIY3_out/"

# Loop through all external devices (TODO: Only loop through sensor cards)
for folder in os.listdir(INPATH):
    # Only look in folders (might be unnecessary, I don't know if regular files
    # can show up here)
    #if os.path.isdir(folder):
    # Loop through all files in the root folder of the device
    for fname in os.listdir(INPATH + folder + "/"):
        # Get the name and extension, i.e. name.ext
        name, ext = os.path.splitext(fname)
        # Only use .data files (there may be config or other junk files on
        # the device which we don't want to process).
        if (ext == ".data"):
            # Open the file as a binary file and read it.
            with open(INPATH + folder + "/" + fname, "rb") as file:
                raw = file.read()

            # Dictionary to store unpacked data. We convert this to a 
            # DataFrame later.
            unpacked = {"timestamp":[], "pressure":[], "temperature":[]}
            # Loop through the data by bytes.
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

            # Read the data into a pandas DataFraame
            data = pd.DataFrame(unpacked)
            # Write the data to a CSV file with the same name as the raw 
            # data.
            if not os.path.exists(f"{OUTPATH}{folder}/"):
                os.mkdir(f"{OUTPATH}{folder}")
            data.to_csv(f"{OUTPATH}{folder}/{name}.csv", index=False)