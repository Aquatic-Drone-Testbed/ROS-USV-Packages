#!/bin/bash

# Input and output file paths
input_file="radar_data.mp4"
output_file="radar_data_trans.mp4"

# Color to remove (black) in hexadecimal format
color_to_remove="0x000000"

# Similarity and blend thresholds
similarity="0.1"
blend="0.2"

# Run FFmpeg with chromakey filter
ffmpeg -i "$input_file" -vf "chromakey=${color_to_remove}:${similarity}:${blend},format=yuva420p" -c:v libx264 -c:a copy -pix_fmt yuva420p "$output_file"
