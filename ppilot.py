from fileinput import filename
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
os.environ['GDK_DEBUG'] = '3'

DEFAULT_WFB_PORT = 8103
DEFAULT_RETR_PORT = 5000
DEFAULT_VIDEO_PORT = 5600

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
gi.require_version('Gtk', '3.0')

from gi.repository import Gst, Gtk, Gdk, GLib, GstVideo

GlobalLock = threading.Lock()
OSD_Data = {}
do_exit = False
TELEMETRY_PERIOD = 0.25

def mavlink_func(master):
    while not do_exit:
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
    OSD_Data['rx_ant_stats'] = {
        'rssi_0': 0,
        'rssi_1': 0,
        'snr_0': 0,
        'snr_1': 0
    }
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((addr, port))

        buffer = ""
        while not do_exit:
            data = s.recv(4096).decode('utf-8')
            if not data: continue

            buffer += data
            try:
                if "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    obj = json.loads(line)
                    if obj.get('type') == 'rx':
                        if len(obj['rx_ant_stats']) < 2:
                            continue
                        OSD_Data['rx_ant_stats']['rssi_0'] = obj['rx_ant_stats'][0]['rssi_avg']
                        OSD_Data['rx_ant_stats']['rssi_1'] = obj['rx_ant_stats'][1]['rssi_avg']
                        OSD_Data['rx_ant_stats']['snr_0'] = obj['rx_ant_stats'][0]['snr_avg']
                        OSD_Data['rx_ant_stats']['snr_1'] = obj['rx_ant_stats'][1]['snr_avg']
            except json.JSONDecodeError:
                continue

class GstElementError(Exception):
    def __init__(self, plugin):
        super().__init__(f'No such element or plugin "{plugin}"')

class X11Player:
    def __init__(self, video_port):
        Gst.init(None)

        self.window = Gtk.Window(title="X11 Low Latency")
        self.window.connect("destroy", Gtk.main_quit)
        self.window.connect("key-press-event", self.on_key_press)
        self.window.set_default_size(1280, 720)

        self.video_area = Gtk.DrawingArea()
        self.window.add(self.video_area)

        self.window.show_all()
        self.window.fullscreen()
        self.video_area.realize()
        self.video_area.add_events(Gdk.EventMask.POINTER_MOTION_MASK)
        self.video_port = video_port
        print(f"Video port: {self.video_port}")

        pipeline_str = (
            f"udpsrc port={self.video_port} buffer-size=90000 name=source ! "
            "identity name=counter ! "
            "application/x-rtp,media=video,clock-rate=90000,encoding-name=H265,payload=96 ! "
            "rtph265depay ! tee name=t ! queue ! h265parse ! avdec_h265 ! videoconvert ! "
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
        self.bus = self.pipeline.get_bus()
        gdk_window = self.video_area.get_window()
        if hasattr(gdk_window, 'get_xid'):
            xid = gdk_window.get_xid()
            self.sink.set_window_handle(xid)
        else:
            print("Error: Window has no XID even in X11 mode!")
            sys.exit(1)
        self.bus.add_signal_watch()
        self.bus.connect("message", self.on_message)
        overlay = self.pipeline.get_by_name("overlay")
        overlay.connect("draw", self.on_draw)
        self.last_bytes = 0
        self.current_bitrate = 0
        self.bytes_per_sec = 0
        self.last_bitrate_check = time.time()
        self.recording_bin = None
        self.record_pad = None
        self.last_video_pts = 0
        self.subtitle_file = None
        self.rec_start_time = 0
        self.rec_last_time = 0
        self.srt_counter = 0
        self.tee = self.pipeline.get_by_name("t")
        self.counter = self.pipeline.get_by_name("counter")
        tee_src_pad = self.tee.get_static_pad("src_0") # або будь-який активний пад
        tee_src_pad.add_probe(Gst.PadProbeType.BUFFER, self._timestamp_probe)
        if self.counter:
            self.counter.set_property("signal-handoffs", True)

    def make_element(self, plugin, name):
        element = Gst.ElementFactory.make(plugin, name)
        if not element:
            print(f'No such element or plugin "{plugin}"')
            raise GstElementError(plugin)
        return element

    def on_key_press(self, widget, event):
        global do_exit
        keyname = Gdk.keyval_name(event.keyval)

        if keyname == 'r' or keyname == 'R':
            self.toggle_record()
        elif keyname == 'q' or keyname == 'Q':
            do_exit = True
            Gtk.main_quit()

    def toggle_record(self):
        if self.recording_bin != None:
            self.stop_recording()
        else:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f"recording_{timestamp}"
            srt_filename = filename + ".srt"
            filename += ".mkv"
            self.subtitle_file = open(srt_filename, "w", encoding="utf-8")
            self.rec_start_time = self.last_video_pts
            self.srt_counter = 1
            self.rec_last_time = 0
            self.start_recording(filename)

    def start_recording(self, filename):
        if self.recording_bin:
            return
        self.rec_q = self.make_element("queue", "record_queue")
        self.rec_parse = self.make_element("h265parse", "record_parse")
        self.rec_mux = self.make_element("matroskamux", "record_mux")
        self.rec_sink = self.make_element("filesink", "record_sink")

        self.rec_sink.set_property("location", filename)
        self.rec_sink.set_property("sync", False)
        self.rec_parse.set_property("config-interval", -1)

        self.rec_elements = [self.rec_q, self.rec_parse, self.rec_mux, self.rec_sink]
        for el in self.rec_elements:
            self.pipeline.add(el)

        self.rec_q.link(self.rec_parse)
        self.rec_parse.link(self.rec_mux)
        self.rec_mux.link(self.rec_sink)

        for el in self.rec_elements:
            el.sync_state_with_parent()

        template = self.tee.get_pad_template("src_%u")
        self.record_pad = self.tee.request_pad(template, None, None)

        sink_pad = self.rec_q.get_static_pad("sink")
        clock = self.pipeline.get_clock()
        base_time = self.pipeline.get_base_time()
        running_time = clock.get_time() - base_time - 0.1
        sink_pad.set_offset(-running_time)
        print(f"Офсет встановлено: -{running_time / 1e9} сек")
        res = self.record_pad.link(sink_pad)

        if res == Gst.PadLinkReturn.OK:
            self.recording_bin = True
            print(f"Recording: {filename}")
        else:
            print(f"Link error: {res}")
            # Clean on error
            self.tee.release_request_pad(self.record_pad)
            self.pipeline.remove(self.rec_q)
            self.rec_q = None
            self.rec_parse = None
            self.rec_mux = None
            self.rec_sink = None

    def _timestamp_probe(self, pad, info):
        buffer = info.get_buffer()
        if buffer.pts != Gst.CLOCK_TIME_NONE:
            self.last_video_pts = buffer.pts
        return Gst.PadProbeReturn.OK

    def stop_recording(self):
        if not self.recording_bin:
            return
        self.record_pad.add_probe(Gst.PadProbeType.IDLE, self._on_pad_idle)

    def _on_pad_idle(self, pad, info):
        queue_pad = self.rec_q.get_static_pad("sink")
        pad.unlink(queue_pad)
        self.tee.release_request_pad(pad)

        queue_pad.send_event(Gst.Event.new_eos())
        sink_pad = self.pipeline.get_by_name("record_sink").get_static_pad("sink")
        sink_pad.add_probe(Gst.PadProbeType.EVENT_DOWNSTREAM, self.on_pad_event)

        return Gst.PadProbeReturn.REMOVE

    def on_pad_event(self, pad, info):
        event = info.get_event()
        if event.type == Gst.EventType.EOS:
            print("EOS went through filesink!")

            GLib.idle_add(self._finalize_recording)

            # Return DROP to stop EOS from proceeding (if necessary),
            # or PASS to proceed as normal.
            return Gst.PadProbeReturn.PASS

        return Gst.PadProbeReturn.OK

    def on_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS:
            print("Received EOS! File is now finalized.")
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err}")
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old, new, pending = message.parse_state_changed()
#                print(f"State changed from {old.value_nick} to {new.value_nick}")

    def _finalize_recording(self):
        for el in self.rec_elements:
            if el:
                el.set_state(Gst.State.NULL)
                self.pipeline.remove(el)

        self.rec_elements = None
        self.recording_bin = None
        self.rec_start_time = 0
        if self.subtitle_file:
            self.subtitle_file.close()
            self.subtitle_file = None
        print("Recording finalized and cleaned up.")
        return False

    def on_draw(self, overlay, context, timestamp, duration):
        pad = overlay.get_static_pad("sink")
        caps = pad.get_current_caps()
        if not caps:
            print("Can't get caps for overlay!")
            return

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
            rect_width = 330
            rect_height = 180
            margin = 20

            x = video_width - rect_width - margin
            y = margin

            context.set_source_rgba(0, 0, 0, 0.4)
            context.rectangle(x, y, rect_width, rect_height)
            context.fill_preserve()
            context.stroke()

            context.select_font_face("Courier New", 0, 1)
            context.set_font_size(20)
            if self.recording_bin != None:
                context.set_source_rgb(0.9, 0.1, 0.1)
                context.move_to(x + 15, y + 30)
                context.show_text(f"● REC")
                self.write_subtitle_frame()

            context.set_source_rgb(1, 1, 1)
            context.move_to(x + 15, y + 60)
            context.show_text(f"   {self.current_bitrate:.2f} Mbps   {OSD_Data['wifi_freq']}MHz")
            context.move_to(x + 15, y + 90)
            context.show_text(f"        FEC F{OSD_Data['fixed']} L{OSD_Data['errs']}")
            context.move_to(x + 15, y + 130)
            rssi0 = OSD_Data['rx_ant_stats']['rssi_0']
            rssi1 = OSD_Data['rx_ant_stats']['rssi_1']
            snr0 = OSD_Data['rx_ant_stats']['snr_0']
            snr1 = OSD_Data['rx_ant_stats']['snr_1']
            context.show_text(f"{rssi0:3} RSSI {rssi1:3}  {snr0:3} SNR {snr1:3}")
            context.move_to(x + 15, y + 160)
            context.show_text(f"    {OSD_Data['rssi']:3}            {OSD_Data['SNR']:}")

    def format_srt_time(self, seconds):
        td = time.gmtime(seconds)
        ms = int((seconds % 1) * 1000)
        return f"{time.strftime('%H:%M:%S', td)},{ms:03d}"

    def write_subtitle_frame(self):
        if not self.subtitle_file:
            return

        now = (self.last_video_pts - self.rec_start_time) / Gst.SECOND
        end = now + TELEMETRY_PERIOD
        if end - self.rec_last_time < TELEMETRY_PERIOD:
            return
        rssi0 = OSD_Data['rx_ant_stats']['rssi_0']
        snr0 = OSD_Data['rx_ant_stats']['snr_0']
        rssi1 = OSD_Data['rx_ant_stats']['rssi_1']
        snr1 = OSD_Data['rx_ant_stats']['snr_1']
        text = f"{{\\an9}}<font size='18'>   {self.current_bitrate:.2f} Mbps   {OSD_Data['wifi_freq']}MHz\n"
        text += f"        FEC F{OSD_Data['fixed']} L{OSD_Data['errs']}            \n"
        text += f"{rssi0:3} RSSI {rssi1:3}  {snr0:3} SNR {snr1:3}\n"
        text += f"    {OSD_Data['rssi']:3}            {OSD_Data['SNR']:3}     </font>"
        self.subtitle_file.write(f"{self.srt_counter}\n")
        self.subtitle_file.write(f"{self.format_srt_time(now)} --> {self.format_srt_time(end)}\n")
        self.subtitle_file.write(f"{text}\n\n")
        self.subtitle_file.flush()
        self.srt_counter += 1
        self.rec_last_time = end

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
    parser.add_argument('-v', '--video-port', default=DEFAULT_VIDEO_PORT, type=int,
                        action='store', help='Video RTP port')
    args = parser.parse_args()
    OSD_Data = {'rssi': 0, 'noise': 0, 'SNR':0, 'fixed': 0, 'errs': 0,
                'wifi_chan': 0, 'wifi_freq': 0}
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((args.address, args.wfb_port))
        print("Connected to JSON server")

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
        player = X11Player(video_port = args.video_port)
        player.run()
    except Exception as e:
        print(f"Caught exception: {e}")
    do_exit = True
    thread_mavlink.join()
    thread_wfb.join()
