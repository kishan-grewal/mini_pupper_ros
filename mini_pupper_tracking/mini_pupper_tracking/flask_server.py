from flask import Flask, Response
import cv2
import time

def create_flask_app(node):
    app = Flask(__name__)

    @app.route('/')
    def index():
        return "<h2>Mini Pupper Tracking</h2><img src='/video_feed' width='640'>"

    @app.route('/video_feed')
    def video_feed():
        def generate():
            while True:
                time.sleep(0.03)
                with node.frame_lock:
                    frame = node.latest_frame.copy() if node.latest_frame is not None else None
                if frame is None:
                    continue
                success, buffer = cv2.imencode('.jpg', frame)
                if not success:
                    continue
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    return app
