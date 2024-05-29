import os
import subprocess

def create_video_from_images(input_directory, output_file, framerate=24):
    # Construct the FFmpeg command
    ffmpeg_command = [
        'ffmpeg',
        '-framerate', str(framerate),
        '-i', os.path.join(input_directory, 'radar_image%d.png'),
        '-vf', 'scale=trunc(iw/2)*2:trunc(ih/2)*2', # Ensure dimensions are even
        '-c:v', 'prores_ks',
        '-pix_fmt', 'yuva444p10le',
        output_file
    ]
    
    # Run the FFmpeg command
    try:
        subprocess.run(ffmpeg_command, check=True)
        print(f"Video created successfully: {output_file}")
    except subprocess.CalledProcessError as e:
        print(f"Error occurred while creating video: {e}")

# Directory containing the processed images
input_directory = './transparent_images'
output_file = 'output.mov'

# Create the video
create_video_from_images(input_directory, output_file)
