import os
import sys
import time
import threading
import ctypes
import argparse
import json
import socket
from pymavlink import mavutil

os.environ['GDK_BACKEND'] = 'x11'

DEFAULT_WFB_PORT = 8103

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
gi.require_version('Gtk', '3.0')

from gi.repository import Gst, GstVideo, Gtk, Gdk

GlobalLock = threading.Lock()
OSD_Data = {}

def mavlink_func(master):
    while True:
        msg = master.recv_match(blocking=True)
        if not msg:
            continue

        if msg.get_type() != "BAD_DATA":
            if msg.get_type() == 'RADIO_STATUS':
                rssi = ctypes.c_int8(msg.rssi).value
                noise = ctypes.c_int8(msg.noise).value
                fixed = msg.fixed
                rxerrs = msg.rxerrors
                with GlobalLock:
                    OSD_Data['rssi'] = rssi
                    OSD_Data['noise'] = noise
                    OSD_Data['SNR'] = abs(noise - rssi)
                    OSD_Data['fixed'] = fixed
                    OSD_Data['errs'] = rxerrs

def wfb_func(addr, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((addr, port))

        buffer = ""
        while True:
            data = s.recv(4096).decode('utf-8')
            if not data: continue

            buffer += data
            o = {}
            try:
                if "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    obj = json.loads(line)
                    if obj.get('type') == 'rx':
                        if len(obj['rx_ant_stats']) < 2:
                            continue
                        ant_val = {
                            'rssi_0': obj['rx_ant_stats'][0]['rssi_avg'],
                            'rssi_1': obj['rx_ant_stats'][1]['rssi_avg'],
                            'snr_0': obj['rx_ant_stats'][0]['snr_avg'],
                            'snr_1': obj['rx_ant_stats'][1]['snr_avg']
                        }
                        OSD_Data['rx_ant_stats'] = ant_val
            except json.JSONDecodeError:
                continue

class X11Player:
    def __init__(self):
        Gst.init(None)

        self.window = Gtk.Window(title="X11 Low Latency")
        self.window.connect("destroy", Gtk.main_quit)
        self.window.set_default_size(1280, 720)

        self.video_area = Gtk.DrawingArea()
        self.window.add(self.video_area)

        self.window.show_all()
        self.window.fullscreen()
        self.video_area.realize()
        self.video_area.add_events(Gdk.EventMask.POINTER_MOTION_MASK)

        pipeline_str = (
            "udpsrc port=5600 buffer-size=90000 name=source ! "
            "identity name=counter ! "
            "application/x-rtp,media=video,clock-rate=90000,encoding-name=H265,payload=96 ! "
            "rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! "
            "video/x-raw,format=BGRx ! "
            "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=2 ! "
            "cairooverlay name=overlay ! "
            "videoconvert ! "
            "xvimagesink name=sink sync=false async=false"
        )

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.sink = self.pipeline.get_by_name("sink")
        self.last_bytes = 0
        self.last_time = time.time()
        gdk_window = self.video_area.get_window()
        if hasattr(gdk_window, 'get_xid'):
            xid = gdk_window.get_xid()
            self.sink.set_window_handle(xid)
        else:
            print("Error: Window has no XID even in X11 mode!")
            sys.exit(1)

        overlay = self.pipeline.get_by_name("overlay")
        overlay.connect("draw", self.on_draw)
        self.last_bytes = 0
        self.current_bitrate = 0
        self.bytes_per_sec = 0
        self.last_bitrate_check = time.time()
        self.counter = self.pipeline.get_by_name("counter")
        if self.counter:
            self.counter.set_property("signal-handoffs", True)

    def on_draw(self, overlay, context, timestamp, duration):
        pad = overlay.get_static_pad("sink")
        caps = pad.get_current_caps()
        if not caps: return

        with GlobalLock:
            video_width = caps.get_structure(0).get_value("width")
            now = time.time()
            dt = now - self.last_bitrate_check
            if dt >= 1.0:
                stats = self.counter.get_property("stats")
                if stats:
                    success, total_bytes = stats.get_uint64("num-bytes")
                    if success:
                        now = time.time()
                        dt = now - self.last_time
                        if dt > 0.5:
                            bytes_diff = total_bytes - self.last_bytes
                            self.current_bitrate = (bytes_diff * 8) / (1024 * 1024 * dt)
                            self.last_bytes = total_bytes
                            self.last_time = now
            rect_width = 320
            rect_height = 150
            margin = 20

            x = video_width - rect_width - margin
            y = margin

            context.set_source_rgba(0, 0, 0, 0.4)
            context.rectangle(x, y, rect_width, rect_height)
            context.fill_preserve()

#            context.set_source_rgb(0, 1, 0.5)
#            context.set_line_width(2)
            context.stroke()

            context.set_source_rgb(1, 1, 1)
            context.select_font_face("Sans", 0, 1)
            context.set_font_size(16)

            context.set_font_size(20)
            context.move_to(x + 15, y + 30)
            context.show_text(f"   {self.current_bitrate:.2f} Mbps   {OSD_Data['wifi_freq']}MHz")
            context.move_to(x + 15, y + 60)
            context.show_text(f"             FEC F{OSD_Data['fixed']} L{OSD_Data['errs']}")
            context.move_to(x + 15, y + 100)
            rssi0 = OSD_Data['rx_ant_stats']['rssi_0']
            rssi1 = OSD_Data['rx_ant_stats']['rssi_1']
            snr0 = OSD_Data['rx_ant_stats']['snr_0']
            snr1 = OSD_Data['rx_ant_stats']['snr_1']
            context.show_text(f" {rssi0:3} RSSI {rssi1:3}  {snr0:3} SNR {snr1:3}")
            context.move_to(x + 15, y + 130)
            context.show_text(f"        {OSD_Data['rssi']:3}                 {OSD_Data['SNR']:}")

    def run(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        Gtk.main()

def get_freq_by_channel(channel):
    # 1. Band 2.4 GHz (Channels 1-13)
    if 1 <= channel <= 13:
        return 2412 + (channel - 1) * 5

    # 2. Channel 14 (Specific for Japan, 2.4 GHz)
    elif channel == 14:
        return 2484

    # 3. Band 5 GHz (Channel 32-177)
    # Formula: 5000 + (channel * 5)
    elif 32 <= channel <= 177:
        return 5000 + (channel * 5)

    # 4. New band 6 GHz (WiFi 6E, channels 1-233)
    # If you use very new equipment
    # Formula: 5940 + (channel * 5)

    else:
        return None

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='ProgramName',
        description='What the program does',
        epilog='Text at the bottom of help')
    parser.add_argument('address', help="Destignation IP address")
    parser.add_argument('-p', '--wfb-port', default=DEFAULT_WFB_PORT, type=int,
                        action='store', help='WFB port')
    args = parser.parse_args()
    OSD_Data = {'rssi': 0, 'noise': 0, 'SNR':0, 'fixed': 0, 'errs': 0,
                'wifi_chan': 0, 'wifi_freq': 0}
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((args.address, args.wfb_port))
        print("Підключено до сервера JSON")

        buffer = ""
        finish = 0
        while True:
            data = s.recv(4096).decode('utf-8')
            if not data: break

            buffer += data
            try:
                if "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    obj = json.loads(line)
                    OSD_Data['wifi_chan'] = obj['settings']['common']['wifi_channel']
                    OSD_Data['wifi_freq'] = get_freq_by_channel(OSD_Data['wifi_chan'])
                    break
            except json.JSONDecodeError:
                break
    OSD_Data['rx_ant_stats'] = []
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    thread_mavlink = threading.Thread(target=mavlink_func, args=(master,))
    thread_mavlink.start()
    thread_wfb = threading.Thread(target=wfb_func, args=(args.address, args.wfb_port,))
    thread_wfb.start()
    try:
        player = X11Player()
        player.run()
    except Exception as e:
        print(f"Caught exception: {e}")
    thread_mavlink.join()
    thread_wfb.join()