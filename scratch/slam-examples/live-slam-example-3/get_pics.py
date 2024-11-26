import cv2
import os
import time

# Settings
output_folder = "calibration_images"  # Folder to save images
image_prefix = "calibration_image"     # Prefix for image filenames
image_format = ".jpg"                  # Image format
capture_interval = 2                   # Interval in seconds between captures

# Create the folder if it doesn't exist
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# GStreamer pipeline for CSI camera on Jetson Orin Nano
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=2,
):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, format=NV12, framerate={framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={display_width}, height={display_height}, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! appsink"
    )

# Initialize the CSI camera with the GStreamer pipeline
cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: Camera could not be opened.")
    exit()

print("Press 'Spacebar' to capture an image or 'q' to quit.")

image_count = 0

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to grab frame.")
        break

    # Display the frame
    cv2.imshow("Camera Calibration", frame)

    # Wait for key press
    key = cv2.waitKey(1) & 0xFF

    # Capture image on spacebar press
    if key == ord(' '):
        image_name = f"{image_prefix}_{image_count}{image_format}"
        image_path = os.path.join(output_folder, image_name)
        cv2.imwrite(image_path, frame)
        print(f"Captured {image_name}")
        image_count += 1
        time.sleep(capture_interval)  # Optional interval for next capture

    # Quit the program on 'q' press
    elif key == ord('q'):
        print("Exiting...")
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()
