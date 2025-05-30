import argparse
import csv
from datetime import datetime, timedelta, timezone
import os
import struct
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

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
                raise ValueError(f"Input data file has an unknown header version string: {header_version_string}")


# GUI Progress Bar
def show_progress_dialog(parent, total_files):
    progress_win = tk.Toplevel(parent)
    progress_win.title("Processing Files")
    progress_win.transient(parent)
    progress_win.grab_set()

    tk.Label(progress_win, text="Processing files...").pack(pady=(10, 0))

    progress_var = tk.DoubleVar()
    progress_bar = ttk.Progressbar(progress_win, length=300, variable=progress_var, maximum=total_files)
    progress_bar.pack(pady=10, padx=20)

    status_label = tk.Label(progress_win, text="Starting...")
    status_label.pack()

    return progress_win, progress_var, progress_bar, status_label


# GUI Functionality
def run_gui():
    last_dir = {"path": os.getcwd()}

    def browse_files():
        files = filedialog.askopenfilenames(
            title="Select .data files",
            filetypes=[("Data Files", "*.data")],
            initialdir=last_dir["path"]
        )
        if files:
            input_var.set(";".join(files))
            last_dir["path"] = os.path.dirname(files[0])

    def browse_output_dir():
        directory = filedialog.askdirectory(
            title="Select Output Directory",
            initialdir=last_dir["path"]
        )
        if directory:
            output_var.set(directory)
            last_dir["path"] = directory

    def process_files():
        input_paths = input_var.get().split(";")
        output_dir = output_var.get()

        if not input_paths or not output_dir:
            messagebox.showerror("Missing Input", "Please specify both input files and output directory.")
            return

        if not os.path.isdir(output_dir):
            messagebox.showerror("Invalid Directory", f"Output directory '{output_dir}' does not exist.")
            return

        total_files = len(input_paths)
        progress_win, progress_var, progress_bar, status_label = show_progress_dialog(root, total_files)
        root.update_idletasks()

        for i, input_path in enumerate(input_paths, 1):
            output_filename = os.path.splitext(os.path.basename(input_path))[0] + ".csv"
            output_path = os.path.join(output_dir, output_filename)

            status_label.config(text=f"Processing file {i} of {total_files}")
            progress_var.set(i)
            progress_bar.update_idletasks()

            read_binary_to_csv(input_path, output_path)

        status_label.config(text="Done!")
        tk.Button(progress_win, text="Close", command=progress_win.destroy).pack(pady=(10, 10))
        progress_bar.update_idletasks()

    def cancel():
        root.destroy()

    root = tk.Tk()
    root.title("Data File to CSV Converter")

    input_var = tk.StringVar()
    output_var = tk.StringVar()

    tk.Label(root, text="Input .data Files:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
    tk.Entry(root, textvariable=input_var, width=50).grid(row=0, column=1, padx=5, pady=5)
    tk.Button(root, text="Browse", command=browse_files).grid(row=0, column=2, padx=5, pady=5)

    tk.Label(root, text="Output Directory:").grid(row=1, column=0, padx=5, pady=5, sticky="e")
    tk.Entry(root, textvariable=output_var, width=50).grid(row=1, column=1, padx=5, pady=5)
    tk.Button(root, text="Browse", command=browse_output_dir).grid(row=1, column=2, padx=5, pady=5)

    tk.Button(root, text="Accept", command=process_files).grid(row=2, column=1, sticky="e", padx=5, pady=10)
    tk.Button(root, text="Cancel", command=cancel).grid(row=2, column=2, sticky="w", padx=5, pady=10)

    root.mainloop()


def main():
    if len(sys.argv) == 1:
        run_gui()
        return

    parser: argparse.ArgumentParser = argparse.ArgumentParser(description="Process raw .data files into .csv files. Run with no arguments to launch a GUI instead.")
    parser.add_argument(
        "input_files",
        nargs="+",
        help="paths to input data files",
    )
    parser.add_argument(
        "-q", "--quiet",
        action="store_true",
        help="suppress processing output",
    )
    parser.add_argument(
        "-o", "--output",
        required=True,
        help="output directory path",
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
