# Script for generating a firmware package

import os
import csv
import json
import zlib

def convert_size(size_str):
    """Convert size string to bytes."""
    if size_str.endswith("K"):
        return int(size_str[:-1]) * 1024
    elif size_str.endswith("M"):
        return int(size_str[:-1]) * 1024 * 1024
    elif size_str.endswith("G"):
        return int(size_str[:-1]) * 1024 * 1024 * 1024
    else:
        return int(size_str)

def read_partition_table(filename):
    partitions = []
    with open(filename, "r") as f:
        data = f.read()
        lines = data.splitlines()
        lines = [line for line in lines if not line.strip().startswith("#")]
        reader = csv.reader(lines)
        for row in reader:
            partition = {
            "name": row[0],
            "type": row[1],
            "subtype": row[2],
            "offset": int(row[3], 16),
            "size": convert_size(row[4]),
            "flags": row[5],
            }
            partitions.append(partition)
    return partitions

def find_partition(partitions, name):
    for partition in partitions:
        if partition["name"] == name:
            return partition
    return None

def calculate_md5(filename):
    """Calculate MD5 checksum of a file."""
    import hashlib
    hash_md5 = hashlib.md5()
    with open(filename, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()

def get_size(filename):
    return os.path.getsize(filename)

def compress(in_file, out_file):
    with open(in_file, "rb") as input_file:
        with open(out_file, "wb") as output_file:
            output_file.write(zlib.compress(input_file.read(), 9))

# Constants
bootloader_offset = 0x0
partition_table_offset = 0x8000
bootloader_path = "build/tanmatsu/bootloader/bootloader.bin"
partition_table_csv_path = "partition_tables/tanmatsu_radio.csv"
partition_table_path = "build/tanmatsu/partition_table/partition-table.bin"
firmware_path = "build/tanmatsu/tanmatsu-radio.bin"

# Generate instructions
partitions = read_partition_table(partition_table_csv_path)
firmware_offset = find_partition(partitions, "ota_0")["offset"]

bootloader_hash = calculate_md5(bootloader_path)
partitions_hash = calculate_md5(partition_table_path)
firmware_hash = calculate_md5(firmware_path)

bootloader_size = get_size(bootloader_path)
partitions_size = get_size(partition_table_path)
firmware_size = get_size(firmware_path)

steps = [
    {"file": "bootloader.zz", "offset": bootloader_offset, "hash": bootloader_hash, "size": bootloader_size, "compressed": True},
    {"file": "partitions.zz", "offset": partition_table_offset, "hash": partitions_hash, "size": partitions_size, "compressed": True},
    {"file": "firmware.zz", "offset": firmware_offset, "hash": firmware_hash, "size": firmware_size, "compressed": True},
]

instructions = {
    "information": {
        "name": "Tanmatsu radio firmware",
        "version": "tbd"
    },
    "steps": steps
}

try:
    os.mkdir("dist")
except:
    pass

# Save instructions to JSON file
output_file = "dist/instructions.trf"
with open(output_file, "w") as f:
    json.dump(instructions, f)

# Compress firmware parts
compress(bootloader_path, "dist/bootloader.zz")
compress(partition_table_path, "dist/partitions.zz")
compress(firmware_path, "dist/firmware.zz")
