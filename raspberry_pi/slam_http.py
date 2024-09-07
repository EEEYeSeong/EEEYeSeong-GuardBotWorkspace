from flask import Flask, send_file
import cv2
import io
import threading
from PIL import Image

app = Flask(__name__)

cap = cv2.VideoCapture(2)
ret, frame = cap.read()

def capture_thread():
    global ret, frame
    
    while True:
        ret, frame = cap.read()

def capture_image():
    if ret:
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(img)

        img_io = io.BytesIO()
        pil_img.save(img_io, 'JPEG')
        img_io.seek(0)
        return img_io
    
    return None


@app.route('/capture', methods=['GET'])
def capture():
    img_io = capture_image()
    if img_io:
        return send_file(img_io, mimetype='image/jpeg')
    return "Failed to capture image", 500

if __name__ == '__main__':
    threading.Thread(target=capture_thread, daemon=True).start()
    app.run(host='0.0.0.0', port=5000)
cap.release()
