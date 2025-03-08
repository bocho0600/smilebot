from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
import time

# Initialize PiCamera2
camera = Picamera2()

# Set up video configuration
frame_height=820
frame_width=616
format='XRGB8888'
video_config = camera.create_video_configuration(main={"format": format, "size": (frame_height, frame_width)})
camera.configure(video_config)

camera.set_controls({"ExposureTime": 70000, "AnalogueGain": 1.0, "ColourGains": (1.4,1.5)})

# Start the camera
camera.start()

# Record the video and save it to a file
encoder = H264Encoder(bitrate=5000000)
encoder.frame_skip_count = 2
output_file = "video_recording.h264"
print(f"Recording video to {output_file}")
camera.start_recording(encoder, output_file)

# Record for 10 seconds
time.sleep(20)

# Stop the recording
camera.stop_recording()

# Stop the camera
camera.stop()

print("Video recording complete!")