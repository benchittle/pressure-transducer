import struct
from datetime import datetime
import pandas as pd
import os

INPATH = "/run/media/benc/C401-E15D/"
OUTPATH = "~/Documents/DIY3_out/"

for fname in os.listdir(INPATH):
    name, ext = os.path.splitext(fname)
    if (ext == ".data"):
        with open(INPATH + fname, "rb") as file:
            raw = file.read()

        unpacked = {"timestamp":[], "pressure":[], "temperature":[]}
        for i in range(0, len(raw), 9):
            timestamp = struct.unpack("<i", raw[i : i + 4])[0]
            unpacked["timestamp"].append(datetime.utcfromtimestamp(timestamp))
            unpacked["pressure"].append(struct.unpack("<f", raw[i + 4 : i + 8])[0])
            unpacked["temperature"].append(struct.unpack("<B", raw[i + 8 : i + 9])[0])

            #print(f"Time: {timestr} \tPressure: {pressure} \tTemperature: {temperature}")

        data = pd.DataFrame(unpacked)
        data.to_csv(OUTPATH + name + ".csv", index=False)
