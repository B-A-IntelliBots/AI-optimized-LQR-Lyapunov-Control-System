'''
Capture Server Node

This ROS service node controls a camera mounted on the robot to capture
environment images for 3D reconstruction.  

Workflow:
1. Uses Picamera2 to capture images.
2. Buffers captured images in memory until instructed to send.
3. Sends images asynchronously to a Telegram bot for remote access.  

Service:
- capture_images (custom service: capture.srv)
    * req.num_images = 0 → capture frame and store in buffer only
    * req.num_images = 1 → capture frame, store it, send all buffered frames to Telegram, then clear buffer
'''

#!/usr/bin/env python3
# capture_server.py

import rospy
import os
import io
import time
import requests
import threading
from control.srv import capture, captureResponse
from picamera2 import Picamera2
from PIL import Image

# ==============================
# Telegram Bot Credentials
# ==============================
BOT_TOKEN = ""   #  Insert bot token
CHAT_ID   = ""   #  Insert target chat ID

# ==============================
# In-memory Image Buffer
# ==============================
captured_images = []  # list of numpy arrays

# ==============================
# Telegram Helper
# ==============================
def send_photo_from_array(img_array):
    """
    Convert numpy array → JPEG (in RAM) → send to Telegram.
    """
    url = f"https://api.telegram.org/bot{BOT_TOKEN}/sendPhoto"

    # Convert to JPEG in memory
    img = Image.fromarray(img_array)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=75)
    buf.seek(0)

    # Send via Telegram API
    files = {"photo": ("image.jpg", buf, "image/jpeg")}
    data  = {"chat_id": CHAT_ID}
    r = requests.post(url, data=data, files=files, timeout=10)

    if r.ok:
        rospy.loginfo("✅ Photo sent successfully")
    else:
        rospy.logwarn(f"⚠️ Telegram error {r.status_code}: {r.text}")

# ==============================
# ROS Service Callback
# ==============================
def handle_capture(req):
    """
    ROS service handler for `capture_images`.

    Commands:
    - req.num_images == 0 → capture frame and store in buffer
    - req.num_images == 1 → capture frame, add to buffer, 
                            send all buffered images to Telegram, 
                            then clear buffer
    """
    global captured_images, picam2

    cmd = req.num_images
    if cmd not in (0, 1):
        rospy.logwarn(f"Unknown command {cmd}. Use 0 (store) or 1 (send).")
        return captureResponse(-1)

    # Capture a frame every call
    img_array = picam2.capture_array()
    captured_images.append(img_array)
    rospy.loginfo(f"📷 Frame captured. Buffer size = {len(captured_images)}")

    # If "send" command → upload all buffered images asynchronously
    if cmd == 1 and captured_images:
        rospy.loginfo("📤 Sending buffered images to Telegram…")

        def _uploader(images):
            """Background uploader thread."""
            for arr in images:
                send_photo_from_array(arr)
                rospy.sleep(0.25)  # throttle requests
            rospy.loginfo("✅ All buffered images sent.")

        # Run upload in background
        threading.Thread(
            target=_uploader,
            args=(captured_images.copy(),),
            daemon=True
        ).start()

        # Clear buffer after handing off to thread
        captured_images.clear()

    return captureResponse(1 if cmd == 1 else 0)

# ==============================
# Main Function
# ==============================
def capture_server():
    """Initialize ROS node and start capture service."""
    rospy.init_node("capturing_server")
    rospy.Service("capture_images", capture, handle_capture)
    rospy.loginfo("📡 Capture service ready.")
    rospy.spin()

# ==============================
# Entry Point
# ==============================
if __name__ == "__main__":
    # Initialize camera
    picam2 = Picamera2()
    picam2.configure(picam2.create_still_configuration())
    picam2.start()
    time.sleep(2)  # warm-up
    capture_server()

