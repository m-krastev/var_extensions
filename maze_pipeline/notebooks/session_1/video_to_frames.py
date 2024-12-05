"""
Extract frames from provided mazeweek1 video (session 1.1)
"""

import cv2
import os

video_path = '../../mazeweek1.mp4'

# Output directory for frames
output_dir = 'frames_mazeweek1'
os.makedirs(output_dir, exist_ok=True)


cap = cv2.VideoCapture(video_path)
frame_count = 0

while cap.isOpened():
    ret, frame = cap.read()  # read one frame at a time
    if not ret: # video over
        break  

    # save frame as an image
    frame_filename = os.path.join(output_dir, f'frame_{frame_count:04d}.jpg')
    cv2.imwrite(frame_filename, frame)
    frame_count += 1

cap.release()
