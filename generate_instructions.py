# Script for generating a JSON file with flashing instructions for OTA updating via the application processor

import os
import csv
import json

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

# Constants
bootloader_offset = 0x0
partition_table_offset = 0x100000

# Generate instructions
partitions = read_partition_table("partition_tables/tanmatsu_radio.csv")
firmware_offset = find_partition(partitions, "ota_0")["offset"]

bootloader_hash = calculate_md5("build/tanmatsu/bootloader/bootloader.bin")
partitions_hash = calculate_md5("build/tanmatsu/partition_table/partition-table.bin")
firmware_hash = calculate_md5("build/tanmatsu/tanmatsu-radio.bin")

bootloader_size = get_size("build/tanmatsu/bootloader/bootloader.bin")
partitions_size = get_size("build/tanmatsu/partition_table/partition-table.bin")
firmware_size = get_size("build/tanmatsu/tanmatsu-radio.bin")

instructions = [
    {"file": "bootloader.zz", "offset": bootloader_offset, "hash": bootloader_hash, "size": bootloader_size},
    {"file": "partition-table.zz", "offset": partition_table_offset, "hash": partitions_hash, "size": partitions_size},
    {"file": "tanmatsu-radio.zz", "offset": firmware_offset, "hash": firmware_hash, "size": firmware_size},
]

# Save instructions to JSON file
output_file = "build/tanmatsu/instructions.json"
with open(output_file, "w") as f:
    json.dump(instructions, f)
