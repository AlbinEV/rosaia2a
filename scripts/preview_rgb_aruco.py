#!/usr/bin/env python3
"""Show RealSense RGB and the eight markers of the two-arm setup."""

import argparse
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import webbrowser

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml


def main():
    """Display RGB without opening the motor board or changing references."""
    config = Path(__file__).resolve().parents[1] / 'ros2_ws/src/aruco_shape_tracker/config/two_arms.yaml'
    settings = yaml.safe_load(config.read_text())['aruco_shape_tracker']['ros__parameters']
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', default=settings['camera_serial'])
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--no-open', action='store_true')
    parser.add_argument('--frames', type=int, default=0, help='Stop after N frames; 0 means unlimited')
    args = parser.parse_args()
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, settings['dictionary']))
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.adaptiveThreshWinSizeMax = settings['adaptive_threshold_window_max']
    parameters.minMarkerPerimeterRate = settings['minimum_marker_perimeter_rate']
    parameters.polygonalApproxAccuracyRate = settings['polygonal_approx_accuracy_rate']
    parameters.errorCorrectionRate = settings['error_correction_rate']
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    fallback_parameters = cv2.aruco.DetectorParameters()
    fallback_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    fallback_parameters.adaptiveThreshWinSizeMin = settings['fallback_threshold_window_size']
    fallback_parameters.adaptiveThreshWinSizeMax = settings['fallback_threshold_window_size']
    fallback_parameters.errorCorrectionRate = settings['error_correction_rate']
    fallback = cv2.aruco.ArucoDetector(dictionary, fallback_parameters)
    expected = set(settings['reference_marker_ids'] + settings['moving_marker_ids'])
    pipeline = rs.pipeline()
    capture = rs.config()
    if args.serial:
        capture.enable_device(args.serial)
    capture.enable_stream(rs.stream.color, settings['width'], settings['height'], rs.format.bgr8, settings['fps'])
    active = False
    server = None
    server_thread = None
    latest_jpeg = [None]
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.split('?')[0] == '/frame.jpg':
                data = latest_jpeg[0]
                if data is None:
                    self.send_error(503, 'Camera starting')
                    return
                content_type = 'image/jpeg'
            elif self.path == '/':
                data = b'''<!doctype html><html><meta charset="utf-8">
                <title>ROSAIA RGB ArUco</title>
                <body style="background:#181818;color:white;font-family:sans-serif">
                <p>ROSAIA RGB / ArUco | Arresta con Ctrl+C nel terminale</p>
                <img id="rgb" style="max-width:100%;height:auto">
                <script>const im=document.getElementById('rgb');
                function refresh(){im.src='/frame.jpg?t='+Date.now();}
                im.onload=()=>setTimeout(refresh,60);
                im.onerror=()=>setTimeout(refresh,500);refresh();</script></body></html>'''
                content_type = 'text/html; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_args):
            pass
    try:
        profile = pipeline.start(capture)
        active = True
        serial = profile.get_device().get_info(rs.camera_info.serial_number)
        if not args.headless:
            server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
            server.daemon_threads = True
            server_thread = threading.Thread(target=server.serve_forever, daemon=True)
            server_thread.start()
            url = f'http://127.0.0.1:{args.port}'
            print(f'Anteprima: {url} | Ctrl+C per arrestare', flush=True)
            if not args.no_open:
                webbrowser.open(url)
        count = 0
        started = time.monotonic()
        previous = None
        while args.frames <= 0 or count < args.frames:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            frame = np.asanyarray(frames.get_color_frame().get_data()).copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found = {}
            for detection in (detector, fallback):
                corners, ids, _ = detection.detectMarkers(gray)
                if ids is not None:
                    for corners_item, marker_id in zip(corners, ids.flatten()):
                        found.setdefault(int(marker_id), corners_item)
                if expected <= set(found):
                    break
            visible = set(found)
            for marker_id, corners_item in found.items():
                color = (0, 220, 255) if marker_id in (0, 1) else (0, 220, 0)
                cv2.aruco.drawDetectedMarkers(frame, [corners_item], np.array([[marker_id]], dtype=np.int32), color)
            count += 1
            rate = count / max(time.monotonic() - started, 1e-6)
            labels = [f'RealSense {serial} | RGB | {rate:.1f} fps | DICT_4X4_50',
                      f'BASE 0: {"OK" if 0 in visible else "MISSING"} | BASE 1: {"OK" if 1 in visible else "MISSING"}',
                      f'Arm 1 [2,3,4]: {sorted(visible & {2,3,4})} | Arm 2 [5,6,7]: {sorted(visible & {5,6,7})}',
                      f'Missing: {sorted(expected - visible)} | Ctrl+C in terminal: exit']
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 130), (0, 0, 0), -1)
            for index, label in enumerate(labels):
                cv2.putText(frame, label, (12, 25 + index * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)
            current = tuple(sorted(visible))
            if current != previous:
                print(f'Visible: {list(current)}; missing: {sorted(expected - visible)}', flush=True)
                previous = current
            if not args.headless:
                ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    latest_jpeg[0] = encoded.tobytes()
    except KeyboardInterrupt:
        pass
    finally:
        if active:
            pipeline.stop()
        if server is not None:
            server.shutdown()
            server.server_close()
            server_thread.join(timeout=2)


if __name__ == '__main__':
    main()
