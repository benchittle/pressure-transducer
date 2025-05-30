import argparse
import csv
from datetime import datetime, timedelta, timezone
import os
import struct

HEADER_VERSION_STRING_SIZE = 4

def read_binary_to_csv(input_file: str, output_csv: str):
    with open(input_file, "rb") as bin_file, open(output_csv, "w", newline="") as csv_file:
        header_version_string = bin_file.read(HEADER_VERSION_STRING_SIZE)
        match header_version_string.decode():
            case "V000":
                # Remaining header size
                HEADER_SIZE = 19
                header = bin_file.read(HEADER_SIZE)

                # Data format: 
                # 16 byte device name (null terminated)
                # 1 byte sampling frequency
                # 9 byte timezone offset string
                # 2 byte chunk size
                header_format = "<16sBH"
                header_unpacked = struct.unpack(header_format, header)

                device_name = header_unpacked[0].decode()
                sample_frequency = header_unpacked[1]
                samples_per_chunk = header_unpacked[2]
                
                writer = csv.writer(csv_file)
                writer.writerow(["device_name", "sample_frequency", "samples_per_chunk"]) 
                writer.writerow([device_name, sample_frequency, samples_per_chunk])
                writer.writerow([])

                # Each chunk is a 4 byte timestamp, a 4 byte temperature reading 
                # (float), and samples_per_chunk pressure readings (floats)
                chunk_size = 4 + 4 + samples_per_chunk * 4
                data_format = f"<I{samples_per_chunk + 1}f"
                sample_interval_ms = 1000 / sample_frequency
                
                writer.writerow(["timestamp", "pressure", "temperature"])

                while chunk := bin_file.read(chunk_size):
                    if len(chunk) < chunk_size:
                        data_format = f"<I{(len(chunk) - 4) // 4}f"

                    unpacked = struct.unpack(data_format, chunk)
                    base_timestamp = unpacked[0]
                    temperature = unpacked[1]
                    pressures = unpacked[2:]

                    base_dt = datetime.fromtimestamp(base_timestamp, timezone.utc)

                    # Write the first row, which has a temperature reading
                    iso_timestamp = base_dt.isoformat(timespec="milliseconds")[:-6]
                    writer.writerow([iso_timestamp, round(pressures[0], 2), round(temperature, 2)])

                    # Write remaining rows without temperature readings
                    for i, pressure in enumerate(pressures[1:], 1):
                        dt = base_dt + timedelta(milliseconds=i * sample_interval_ms)
                        iso_timestamp = dt.isoformat(timespec="milliseconds")[:-6]
                        writer.writerow([iso_timestamp, round(pressure, 2)])
            case _:
                return ValueError(f"Input data file has an unknown header version string: {header_version_string}")


def main():
    parser: argparse.ArgumentParser = argparse.ArgumentParser(description="Process raw .data files into .csv files")
    parser.add_argument(
        "input_files",
        nargs="+",
        help="Paths to input data files",
    )
    parser.add_argument(
        "-o", "--output",
        required=False,
        help="Output directory path",
    )
    parser.add_argument(
        "-q", "--quiet",
        action="store_true",
        help="Suppress processing output",
    )

    args = parser.parse_args()

    # Ensure output directory exists
    if not os.path.isdir(args.output):
        print(f"Output directory '{args.output}' does not exist.")
        exit(1)

    # Example: process each file
    for input_path in args.input_files:
        if not args.quiet:
            print(f"Processing {input_path}...")
        output_filename = os.path.splitext(os.path.basename(input_path))[0] + ".csv"
        output_path = os.path.join(args.output, output_filename)
        read_binary_to_csv(input_path, output_path)
        

if __name__ == "__main__":
    main()
