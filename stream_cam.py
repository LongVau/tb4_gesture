# Lưu file stream_cam.py trên Windows và chạy: python stream_cam.py
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer

cap = cv2.VideoCapture(0)

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            _, jpeg = cv2.imencode('.jpg', frame)
            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

HTTPServer(('0.0.0.0', 5005), StreamHandler).serve_forever()