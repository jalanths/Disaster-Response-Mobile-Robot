#!/usr/bin/env python3
"""
Disaster Rescue Rover — Web Dashboard
Live rover status at http://localhost:5000
Subscribes to: /battery_level, /battery_status, /waypoint_status,
               /rescue_detection/alerts, /rescue_detection/image,
               /camera/image_raw, /object_detection/detections
"""

import json
import base64
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flask import Flask, render_template_string
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')
bridge = CvBridge()

state = {
    'battery_level': '--',
    'battery_status': '--',
    'waypoint': '--',
    'waypoint_progress': '--',
    'survivors_found': 0,
    'zones_cleared': 0,
    'alerts': [],
    'detections': '--',
}

camera_frame = {'raw': None, 'annotated': None}

HTML = """<!DOCTYPE html>
<html>
<head>
  <title>Disaster Rescue Rover Dashboard</title>
  <script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: 'Segoe UI', sans-serif; background: #0d0f0e; color: #e0e0e0; padding: 20px; }
    h1 { text-align: center; color: #ff6b2b; margin-bottom: 24px; letter-spacing: 2px; font-size: 1.6rem; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 16px; }
    .card { background: #161a15; border-radius: 12px; padding: 20px; border: 1px solid #2a3028; }
    .card h2 { font-size: 0.75rem; text-transform: uppercase; letter-spacing: 1px; color: #888; margin-bottom: 12px; }
    .value { font-size: 2rem; font-weight: bold; color: #ff6b2b; }
    .value.safe { color: #4caf50; }
    .badge { display: inline-block; padding: 4px 12px; border-radius: 20px; font-size: 0.85rem; font-weight: bold; margin-top: 8px; }
    .OK       { background: #1a3a1a; color: #4caf50; }
    .LOW      { background: #3a2a00; color: #ff9800; }
    .CRITICAL { background: #3a0000; color: #f44336; }
    .bar-bg { background: #2a3028; border-radius: 8px; height: 12px; margin-top: 10px; overflow: hidden; }
    .bar    { height: 100%; border-radius: 8px; transition: width 0.5s, background 0.5s; }
    .stat-row { display: flex; gap: 16px; margin-top: 10px; }
    .stat-box { flex: 1; background: #1e2420; border-radius: 8px; padding: 10px; text-align: center; }
    .stat-box .num { font-size: 1.6rem; font-weight: bold; color: #ff6b2b; }
    .stat-box .lbl { font-size: 0.7rem; color: #888; text-transform: uppercase; margin-top: 2px; }
    .alerts-list { max-height: 220px; overflow-y: auto; }
    .alert-item { background: #2a1a10; border-left: 3px solid #ff6b2b; padding: 8px 12px;
                  margin-bottom: 6px; border-radius: 4px; font-size: 0.82rem; line-height: 1.5; }
    .alert-item.CRITICAL { border-color: #f44336; background: #2a1010; }
    .alert-item.HIGH     { border-color: #ff9800; background: #2a1f10; }
    .alert-item.MEDIUM   { border-color: #ff6b2b; background: #2a1a10; }
    .cam-feed { width: 100%; border-radius: 8px; background: #111; min-height: 160px;
                display: flex; align-items: center; justify-content: center; }
    .cam-feed img { width: 100%; border-radius: 8px; display: block; }
    .cam-placeholder { color: #444; font-size: 0.85rem; }
    .tab-btn { background: #2a3028; border: none; color: #888; padding: 6px 14px;
               border-radius: 6px; cursor: pointer; margin-right: 6px; font-size: 0.8rem; }
    .tab-btn.active { background: #ff6b2b; color: #000; font-weight: bold; }
    .dot { width: 8px; height: 8px; border-radius: 50%; display: inline-block; margin-right: 6px; }
    .dot.on  { background: #4caf50; box-shadow: 0 0 6px #4caf50; }
    .dot.off { background: #555; }
    .mission-badge { display: inline-block; background: #1e2f1e; color: #4caf50;
                     border: 1px solid #4caf50; border-radius: 6px; padding: 3px 10px;
                     font-size: 0.75rem; margin-top: 6px; }
  </style>
</head>
<body>
  <h1>&#x1F6A8; Disaster Rescue Rover</h1>
  <div class="grid">

    <!-- Battery -->
    <div class="card">
      <h2>Battery</h2>
      <div class="value" id="bat-level">--</div>
      <div class="bar-bg"><div class="bar" id="bat-bar" style="width:0%"></div></div>
      <span class="badge" id="bat-status">--</span>
    </div>

    <!-- Rescue Stats -->
    <div class="card">
      <h2>Rescue Status</h2>
      <div class="stat-row">
        <div class="stat-box">
          <div class="num" id="survivors-count">0</div>
          <div class="lbl">Survivors Found</div>
        </div>
        <div class="stat-box">
          <div class="num safe" id="zones-count">0</div>
          <div class="lbl">Zones Cleared</div>
        </div>
      </div>
    </div>

    <!-- Current Zone -->
    <div class="card">
      <h2>Current Zone</h2>
      <div class="value" id="wp-name" style="font-size:1.4rem;">--</div>
      <div style="color:#888; margin-top:8px; font-size:0.9rem;">Progress: <span id="wp-progress">--</span></div>
      <div class="mission-badge" id="mission-badge">STANDBY</div>
    </div>

    <!-- Detections -->
    <div class="card">
      <h2>Object Detection</h2>
      <div id="detections" style="font-size:0.9rem; line-height:1.8; color:#ccc;">--</div>
    </div>

    <!-- Camera -->
    <div class="card" style="grid-column: span 2;">
      <h2>
        Camera Feed &nbsp;
        <button class="tab-btn active" onclick="switchCam('raw', this)">Raw</button>
        <button class="tab-btn" onclick="switchCam('annotated', this)">Rescue Annotated</button>
      </h2>
      <div class="cam-feed">
        <img id="cam-img" src="" alt="" style="display:none"/>
        <span class="cam-placeholder" id="cam-placeholder">Waiting for camera...</span>
      </div>
    </div>

    <!-- Alerts -->
    <div class="card" style="grid-column: span 2;">
      <h2>Rescue Alerts <span id="alert-count" style="color:#f44336"></span></h2>
      <div class="alerts-list" id="alerts-list"><div style="color:#555; font-size:0.85rem;">No alerts yet.</div></div>
    </div>

  </div>

  <p style="text-align:center; margin-top:20px; font-size:0.75rem; color:#444;">
    <span class="dot" id="conn-dot"></span><span id="conn-text">Connecting...</span>
  </p>

  <script>
    const socket = io();
    let activeCam = 'raw';

    function switchCam(type, btn) {
      activeCam = type;
      document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
    }

    socket.on('connect', () => {
      document.getElementById('conn-dot').className = 'dot on';
      document.getElementById('conn-text').textContent = 'Connected to rescue rover';
    });
    socket.on('disconnect', () => {
      document.getElementById('conn-dot').className = 'dot off';
      document.getElementById('conn-text').textContent = 'Disconnected';
    });

    socket.on('state', (data) => {
      // Battery
      const lvl = parseFloat(data.battery_level);
      document.getElementById('bat-level').textContent =
        isNaN(lvl) ? '--' : lvl.toFixed(1) + '%';
      const bar = document.getElementById('bat-bar');
      bar.style.width = (isNaN(lvl) ? 0 : lvl) + '%';
      bar.style.background = lvl > 50 ? '#4caf50' : lvl > 20 ? '#ff9800' : '#f44336';
      const st = data.battery_status;
      const badge = document.getElementById('bat-status');
      badge.textContent = st;
      badge.className = 'badge ' + st;

      // Rescue stats
      document.getElementById('survivors-count').textContent = data.survivors_found || 0;
      document.getElementById('zones-count').textContent = data.zones_cleared || 0;

      // Current zone
      const wpName = data.waypoint;
      document.getElementById('wp-name').textContent = wpName;
      document.getElementById('wp-progress').textContent = data.waypoint_progress;
      const mb = document.getElementById('mission-badge');
      if (wpName && wpName !== '--') {
        mb.textContent = wpName.replace(/_/g, ' ').toUpperCase();
        mb.style.borderColor = '#ff6b2b';
        mb.style.color = '#ff6b2b';
        mb.style.background = '#2a1a10';
      }

      // Detections
      document.getElementById('detections').innerHTML =
        data.detections === '--' ? '<span style="color:#555">No detections</span>' :
        data.detections.split('\\n').map(l => '<div>' + l + '</div>').join('');

      // Alerts
      const list = document.getElementById('alerts-list');
      if (data.alerts.length === 0) {
        list.innerHTML = '<div style="color:#555; font-size:0.85rem;">No alerts yet.</div>';
        document.getElementById('alert-count').textContent = '';
      } else {
        document.getElementById('alert-count').textContent = '(' + data.alerts.length + ')';
        list.innerHTML = data.alerts.slice().reverse().map(a => {
          const sev = a.severity || 'MEDIUM';
          const typeIcon = a.type === 'SURVIVOR' ? '&#x1F9D1;' :
                           a.type === 'FIRE'     ? '&#x1F525;' :
                           a.type === 'HAZMAT'   ? '&#x26A0;' : '&#x1F9F1;';
          return `<div class="alert-item ${sev}">
            ${typeIcon} <strong>${a.timestamp || ''}</strong> &nbsp;
            <strong style="color:${sev==='CRITICAL'?'#f44336':sev==='HIGH'?'#ff9800':'#ff6b2b'}">${sev}</strong>
            &nbsp;— ${a.type || ''}: ${a.label || ''} &nbsp;|&nbsp; ${a.action || ''}
          </div>`;
        }).join('');
      }
    });

    socket.on('camera', (data) => {
      const frame = activeCam === 'annotated' ? data.annotated : data.raw;
      const img = document.getElementById('cam-img');
      const ph  = document.getElementById('cam-placeholder');
      if (frame) {
        img.src = 'data:image/jpeg;base64,' + frame;
        img.style.display = 'block';
        ph.style.display  = 'none';
      }
    });
  </script>
</body>
</html>"""


def img_to_b64(cv_img, width=640):
    h, w = cv_img.shape[:2]
    scale = width / w
    resized = cv2.resize(cv_img, (width, int(h * scale)))
    _, buf = cv2.imencode('.jpg', resized, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return base64.b64encode(buf).decode('utf-8')


class DashboardNode(Node):
    def __init__(self):
        super().__init__('web_dashboard')
        self.create_subscription(Float32, '/battery_level',              self._bat_level,  10)
        self.create_subscription(String,  '/battery_status',             self._bat_status, 10)
        self.create_subscription(String,  '/waypoint_status',            self._wp_status,  10)
        self.create_subscription(String,  '/rescue_detection/alerts',    self._alert,      10)
        self.create_subscription(String,  '/object_detection/detections',self._detection,  10)
        self.create_subscription(Image,   '/camera/image_raw',           self._cam_raw,    1)
        self.create_subscription(Image,   '/rescue_detection/image',     self._cam_ann,    1)
        self.get_logger().info('Rescue rover dashboard ready. Open http://localhost:5000')

    def _bat_level(self, msg):
        state['battery_level'] = round(msg.data, 1)
        self._push()

    def _bat_status(self, msg):
        state['battery_status'] = msg.data
        self._push()

    def _wp_status(self, msg):
        try:
            d = json.loads(msg.data)
            state['waypoint']          = d.get('name', '--')
            state['waypoint_progress'] = d.get('progress', '--')
            state['survivors_found']   = d.get('survivors_found', state['survivors_found'])
            state['zones_cleared']     = d.get('zones_cleared',   state['zones_cleared'])
        except Exception:
            state['waypoint'] = msg.data
        self._push()

    def _alert(self, msg):
        try:
            alert = json.loads(msg.data)
            state['alerts'].append(alert)
            if alert.get('type') == 'SURVIVOR':
                state['survivors_found'] = state.get('survivors_found', 0) + 1
            if len(state['alerts']) > 50:
                state['alerts'] = state['alerts'][-50:]
        except Exception:
            pass
        self._push()

    def _detection(self, msg):
        state['detections'] = msg.data
        self._push()

    def _cam_raw(self, msg):
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, 'bgr8')
            camera_frame['raw'] = img_to_b64(cv_img)
            self._push_camera()
        except Exception:
            pass

    def _cam_ann(self, msg):
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, 'bgr8')
            camera_frame['annotated'] = img_to_b64(cv_img)
            self._push_camera()
        except Exception:
            pass

    def _push(self):
        socketio.emit('state', state)

    def _push_camera(self):
        socketio.emit('camera', camera_frame)


@app.route('/')
def index():
    return render_template_string(HTML)


def ros_thread():
    rclpy.init()
    node = DashboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    t = threading.Thread(target=ros_thread, daemon=True)
    t.start()
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False)
