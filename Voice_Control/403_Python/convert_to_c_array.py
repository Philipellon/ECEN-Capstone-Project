# convert_to_c_array.py

'''
This code converts a basic model to a c array

import sys

if len(sys.argv) < 2:
    print("Usage: python convert_to_c_array.py <model.tflite>")
    sys.exit(1)

infile = sys.argv[1]
outfile = "model_data.cc"

with open(infile, "rb") as f:
    data = f.read()

with open(outfile, "w") as f:
    f.write("unsigned char model_data[] = {\n")

    # Write data in rows of 12 bytes for readability
    for i, byte in enumerate(data):
        f.write(f"0x{byte:02x},")
        if (i + 1) % 12 == 0:
            f.write("\n")
        else:
            f.write(" ")

    f.write("\n};\n")
    f.write(f"unsigned int model_data_len = {len(data)};\n")

print(f"Converted '{infile}' to C array in '{outfile}'.")

'''
# Converts a .quant file to c array

import sys
import os

if len(sys.argv) < 2:
    print("Usage: python convert_to_c_array.py <model.tflite>")
    sys.exit(1)

infile = sys.argv[1]
# Use the input file name (without extension) to create a unique output file name and variable names
base_name = os.path.splitext(os.path.basename(infile))[0]
outfile = f"{base_name}_data.cc"

with open(infile, "rb") as f:
    data = f.read()

with open(outfile, "w") as f:
    f.write(f"unsigned char {base_name}_data[] = {{\n")

    # Write data in rows of 12 bytes for readability
    for i, byte in enumerate(data):
        f.write(f"0x{byte:02x},")
        if (i + 1) % 12 == 0:
            f.write("\n")
        else:
            f.write(" ")

    f.write("\n};\n")
    f.write(f"unsigned int {base_name}_data_len = {len(data)};\n")

print(f"Converted '{infile}' to C array in '{outfile}'.")

