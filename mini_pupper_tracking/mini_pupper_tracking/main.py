#!/usr/bin/env python3

import rclpy
from threading import Thread
from mini_pupper_tracking.tracking_node import TrackingNode
from mini_pupper_tracking.flask_server import create_flask_app
import os

def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()

    # Start Flask in background
    app = create_flask_app(node)
    flask_thread = Thread(target=lambda: app.run(
        host="0.0.0.0", port=5000,
        debug=False, use_reloader=False, threaded=True), daemon=True)
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
