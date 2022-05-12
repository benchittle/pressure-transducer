import struct
from datetime import datetime

FILEPATH = "/run/media/benc/C401-E15D/TEST"

with open(FILEPATH, "rb") as file:
    data = file.read()

for i in range(0, len(data), 9):
    timestamp = struct.unpack("<i", data[i : i + 4])[0]
    timestr = datetime.utcfromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')
    pressure = struct.unpack("<f", data[i + 4 : i + 8])[0]
    temperature = struct.unpack("<B", data[i + 8 : i + 9])[0]

print(f"Time: {timestr}\nPressure: {pressure}\nTemperature: {temperature}")