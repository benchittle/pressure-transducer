# %%
import glob
import os

import pandas as pd

import processing_tools as pt

# %% ###########################################################################
#                           Your paths here
RAW_PATH = r"/path/to/raw/folder"
UNPACKED_PATH = r"/path/to/unpacking/folder"
################################################################################

# %% Read all raw data into processed csv files.
print(f"Reading raw data from {RAW_PATH} and unpacking into {UNPACKED_PATH}")
pt.read_raw(RAW_PATH, UNPACKED_PATH)