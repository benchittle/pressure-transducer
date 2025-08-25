#!/usr/bin/env python3

import os
import sys
import zipfile

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 script.py <hardware_target>")
        sys.exit(1)

    hardware_target = sys.argv[1]
    base_dir = os.path.dirname(os.path.realpath(__file__))
    build_dir = os.path.join(base_dir, "..", "transducer-firmware", "build")

    bootloader_path = os.path.join(build_dir, "bootloader", "bootloader.bin")
    firmware_path = os.path.join(build_dir, "transducer-firmware.bin")
    partition_table_path = os.path.join(build_dir, "partition_table", "partition-table.bin")

    zip_filename = os.path.join(build_dir, f"{hardware_target}_firmware.zip")

    # Check files exist
    for path in [bootloader_path, firmware_path, partition_table_path]:
        if not os.path.exists(path):
            print(f"Error: File not found: {path}")
            sys.exit(1)

    # Create the zip archive
    with zipfile.ZipFile(zip_filename, 'w') as zipf:
        zipf.write(bootloader_path, arcname="bootloader.bin")
        zipf.write(firmware_path, arcname=f"{hardware_target}.bin")
        zipf.write(partition_table_path, arcname="partition-table.bin")

    print(f"Created archive: {os.path.abspath(zip_filename)}")

if __name__ == "__main__":
    main()
