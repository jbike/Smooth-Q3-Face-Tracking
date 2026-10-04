import os
import time
import threading
import sys
import termios
import tty
import select

os.environ["GST_PLUGIN_FEATURE_RANK"] = "vaapidecodebin:NONE"

import gi
import setproctitle
from bluepy import btle

gi.require_version("Gst", "1.0")

import hailo

from hailo_apps.python.core.common.core import (
    get_pipeline_parser,
    get_resource_path,
)

from hailo_apps.python.core.common.defines import (
    RESOURCES_SO_DIR_NAME,
    RESOURCES_JSON_DIR_NAME,
    FACE_DETECTION_POSTPROCESS_SO_FILENAME,
    FACE_DETECTION_JSON_NAME,
    SCRFD_10G_POSTPROCESS_FUNCTION,
)

from hailo_apps.python.core.common.hailo_logger import get_logger

from hailo_apps.python.core.gstreamer.gstreamer_app import (
    GStreamerApp,
    app_callback_class,
)

from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
    INFERENCE_PIPELINE,
    INFERENCE_PIPELINE_WRAPPER,
    USER_CALLBACK_PIPELINE,
    DISPLAY_PIPELINE,
)

hailo_logger = get_logger(__name__)

FACE_HEF = "/usr/local/hailo/resources/models/hailo8/scrfd_10g.hef"


# ==========================================================
# SMOOTH-Q3 BLE
# ==========================================================

Q3_ADDRESS = "D5:4D:56:D7:F2:DA"

WRITE_HANDLE = 51
NOTIFY_CCCD_HANDLE = 54

Q3_CENTER = 512

q3_seq = 0x61
q3 = None


# ==========================================================
# FACE TRACKING SETTINGS
# ==========================================================

TARGET_X = 0.50
TARGET_Y = 0.35

DEADBAND_X = 0.035
DEADBAND_Y = 0.045

FACE_MIN_CONF = 0.40
FACE_HOLD_TIME = 0.20

CONTROL_INTERVAL = 0.05

Q3_YAW_GAIN = 700.0
Q3_PITCH_GAIN = 600.0

Q3_YAW_MIN = 300
Q3_YAW_MAX = 724

Q3_PITCH_MIN = 320
Q3_PITCH_MAX = 704

Q3_YAW_SIGN = 1.0
Q3_PITCH_SIGN = 1.0


# ==========================================================
# MANUAL CENTERING SETTINGS
# ==========================================================

MANUAL_YAW_LEFT = 375
MANUAL_YAW_RIGHT = 650

MANUAL_PITCH_UP = 375
MANUAL_PITCH_DOWN = 650

MANUAL_BURST_COUNT = 3
MANUAL_BURST_DELAY = 0.08


# ==========================================================
# SHARED STATE
# ==========================================================

latest_face_x = None
latest_face_y = None
latest_face_conf = 0.0
latest_face_time = 0.0

state_lock = threading.Lock()
running = True


# ==========================================================
# Q3 PACKETS
# ==========================================================

def crc16_xmodem(data):

    crc = 0

    for byte in data:

        crc ^= byte << 8

        for _ in range(8):

            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


def make_q3_packet(cmd, value, flag=0x10):

    global q3_seq

    value = int(value)

    lo = value & 0xFF
    hi = (value >> 8) & 0xFF

    pkt = bytearray([
        0x24, 0x3C,
        0x08, 0x00,
        0x18, 0x12,
        q3_seq & 0xFF,
        0x01,
        cmd,
        flag,
        lo,
        hi,
    ])

    crc = crc16_xmodem(pkt[4:12])

    pkt.append(crc & 0xFF)
    pkt.append((crc >> 8) & 0xFF)

    q3_seq = (q3_seq + 1) & 0xFF

    return bytes(pkt)


def H(s):
    return bytes.fromhex(s)


INIT_60 = H(
    "24 3c 08 00 18 12 60 01 04 00 00 00 4e 4b"
)

INIT_5F = H(
    "24 3c 08 00 18 12 5f 01 04 00 00 00 c1 a1"
)

INFO = H(
    "24 3c 03 00 18 18 13 4a 42"
)

HEARTBEAT = H(
    "24 3c 04 00 18 18 10 00 d5 77"
)


def q3_send_raw(pkt):

    if q3 is None:
        return

    q3.writeCharacteristic(
        WRITE_HANDLE,
        pkt,
        withResponse=False,
    )


def q3_send_position(yaw, pitch):

    q3_send_raw(
        make_q3_packet(
            0x0A,
            yaw
        )
    )

    time.sleep(0.01)

    q3_send_raw(
        make_q3_packet(
            0x09,
            pitch
        )
    )


def q3_enable_tracking():

    q3_send_raw(
        make_q3_packet(
            0x0B,
            100
        )
    )


# ==========================================================
# CONNECT
# ==========================================================

def connect_q3():

    global q3

    print("Connecting to Smooth-Q3...")

    q3 = btle.Peripheral(
        Q3_ADDRESS,
        addrType=btle.ADDR_TYPE_RANDOM,
    )

    print("Q3 connected.")

    q3.writeCharacteristic(
        NOTIFY_CCCD_HANDLE,
        b"\x01\x00",
        withResponse=True,
    )

    time.sleep(0.30)

    print("Initializing Q3...")

    q3_send_raw(INIT_60)
    time.sleep(0.10)

    q3_send_raw(INIT_5F)
    time.sleep(0.10)

    q3_send_raw(INFO)
    time.sleep(0.20)

    q3_send_raw(HEARTBEAT)
    time.sleep(0.20)

    print("Q3 initialized.")


# ==========================================================
# MANUAL CENTERING
# ==========================================================

def manual_burst(yaw, pitch):

    for _ in range(MANUAL_BURST_COUNT):

        q3_send_position(
            yaw,
            pitch
        )

        q3.waitForNotifications(0.03)

        time.sleep(
            MANUAL_BURST_DELAY
        )


def manual_centering():

    print()
    print("==============================")
    print(" MANUAL GIMBAL POSITIONING")
    print("==============================")
    print()
    print("A = pan left")
    print("D = pan right")
    print("W = tilt up")
    print("S = tilt down")
    print("ENTER = accept position and start tracking")
    print("Q = quit")
    print()

    # Tracking command mode must be active
    # for the manual directional commands to work.
    q3_enable_tracking()
    time.sleep(0.20)

    old_settings = termios.tcgetattr(sys.stdin)

    try:

        tty.setcbreak(
            sys.stdin.fileno()
        )

        last_heartbeat = time.monotonic()

        while True:

            if (
                time.monotonic()
                - last_heartbeat
                >= 0.5
            ):

                q3_send_raw(
                    HEARTBEAT
                )

                last_heartbeat = (
                    time.monotonic()
                )


            ready, _, _ = select.select(
                [sys.stdin],
                [],
                [],
                0.05
            )

            if not ready:
                q3.waitForNotifications(0.01)
                continue


            key = sys.stdin.read(1)


            # ENTER
            if key in ("\n", "\r"):

                print()
                print(
                    "Starting automatic face tracking..."
                )

                break


            key = key.lower()


            if key == "a":

                manual_burst(
                    MANUAL_YAW_LEFT,
                    Q3_CENTER
                )


            elif key == "d":

                manual_burst(
                    MANUAL_YAW_RIGHT,
                    Q3_CENTER
                )


            elif key == "w":

                manual_burst(
                    Q3_CENTER,
                    MANUAL_PITCH_UP
                )


            elif key == "s":

                manual_burst(
                    Q3_CENTER,
                    MANUAL_PITCH_DOWN
                )


            elif key == "q":

                return False


    finally:

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_settings
        )


    # Send one neutral tracking error after positioning.
    q3_send_position(
        Q3_CENTER,
        Q3_CENTER
    )

    time.sleep(0.20)

    return True


# ==========================================================
# CONTROL LOOP
# ==========================================================

def control_loop():

    global running

    next_time = time.monotonic()

    last_heartbeat = time.monotonic()
    last_print = 0.0

    had_face = False


    while running:

        now = time.monotonic()

        if now < next_time:
            time.sleep(
                next_time - now
            )

        now = time.monotonic()

        next_time = (
            now + CONTROL_INTERVAL
        )


        with state_lock:

            face_x = latest_face_x
            face_y = latest_face_y
            face_conf = latest_face_conf

            if latest_face_time > 0:
                face_age = (
                    now
                    - latest_face_time
                )
            else:
                face_age = 999.0


        if (
            now
            - last_heartbeat
            >= 0.5
        ):

            try:

                q3_send_raw(
                    HEARTBEAT
                )

            except Exception as e:

                print(
                    f"Q3 heartbeat error: {e}"
                )

            last_heartbeat = now


        face_valid = (
            face_x is not None
            and face_y is not None
            and face_age <= FACE_HOLD_TIME
        )


        # No face: send NOTHING.
        if not face_valid:

            if had_face:
                print(
                    "FACE LOST - holding position"
                )

            had_face = False
            continue


        if not had_face:

            print(
                "FACE ACQUIRED"
            )


        had_face = True


        error_x = (
            face_x - TARGET_X
        )

        error_y = (
            face_y - TARGET_Y
        )


        if abs(error_x) < DEADBAND_X:
            error_x = 0.0

        if abs(error_y) < DEADBAND_Y:
            error_y = 0.0


        q3_yaw = (
            Q3_CENTER
            + Q3_YAW_SIGN
            * error_x
            * Q3_YAW_GAIN
        )

        q3_pitch = (
            Q3_CENTER
            + Q3_PITCH_SIGN
            * error_y
            * Q3_PITCH_GAIN
        )


        q3_yaw = max(
            Q3_YAW_MIN,
            min(
                Q3_YAW_MAX,
                q3_yaw
            )
        )

        q3_pitch = max(
            Q3_PITCH_MIN,
            min(
                Q3_PITCH_MAX,
                q3_pitch
            )
        )


        q3_yaw = int(
            q3_yaw
        )

        q3_pitch = int(
            q3_pitch
        )


        try:

            q3_send_position(
                q3_yaw,
                q3_pitch
            )

        except Exception as e:

            print(
                f"Q3 BLE error: {e}"
            )


        if (
            now
            - last_print
            >= 0.25
        ):

            print(
                f"FACE "
                f"X={face_x:.3f} "
                f"Y={face_y:.3f} "
                f"errX={error_x:+.3f} "
                f"errY={error_y:+.3f} "
                f"yaw={q3_yaw} "
                f"pitch={q3_pitch} "
                f"conf={face_conf:.2f}"
            )

            last_print = now


# ==========================================================
# HAILO APP
# ==========================================================

class user_app_callback_class(
    app_callback_class
):

    def __init__(self):
        super().__init__()


class GStreamerFaceTrackApp(
    GStreamerApp
):

    def __init__(
        self,
        app_callback,
        user_data,
        parser=None,
    ):

        if parser is None:

            parser = (
                get_pipeline_parser()
            )


        super().__init__(
            parser,
            user_data,
        )


        self.face_hef = FACE_HEF


        self.face_post_so = (
            get_resource_path(
                pipeline_name=None,
                resource_type=
                    RESOURCES_SO_DIR_NAME,
                arch=self.arch,
                model=
                    FACE_DETECTION_POSTPROCESS_SO_FILENAME,
            )
        )


        self.face_json = (
            get_resource_path(
                pipeline_name=None,
                resource_type=
                    RESOURCES_JSON_DIR_NAME,
                arch=self.arch,
                model=
                    FACE_DETECTION_JSON_NAME,
            )
        )


        self.face_function = (
            SCRFD_10G_POSTPROCESS_FUNCTION
        )


        self.app_callback = (
            app_callback
        )


        setproctitle.setproctitle(
            "Hailo Q3 Face Tracker"
        )


        self.create_pipeline()


    def get_pipeline_string(self):

        source_pipeline = (
            self.get_source_pipeline(
                no_webcam_compression=True
            )
        )


        face_inference = (
            INFERENCE_PIPELINE(
                hef_path=
                    self.face_hef,
                post_process_so=
                    self.face_post_so,
                post_function_name=
                    self.face_function,
                batch_size=1,
                config_json=
                    self.face_json,
                name=
                    "face_inference",
            )
        )


        face_wrapper = (
            INFERENCE_PIPELINE_WRAPPER(
                face_inference,
                name=
                    "face_wrapper",
            )
        )


        callback_pipeline = (
            USER_CALLBACK_PIPELINE()
        )


        display_pipeline = (
            DISPLAY_PIPELINE(
                video_sink=
                    self.video_sink,
                sync=
                    self.sync,
                show_fps=
                    self.show_fps,
            )
        )


        return (
            f"{source_pipeline} ! "
            f"{face_wrapper} ! "
            f"{callback_pipeline} ! "
            f"{display_pipeline}"
        )


# ==========================================================
# FACE CALLBACK
# ==========================================================

def app_callback(
    element,
    buffer,
    user_data,
):

    global latest_face_x
    global latest_face_y
    global latest_face_conf
    global latest_face_time


    if buffer is None:
        return


    roi = (
        hailo.get_roi_from_buffer(
            buffer
        )
    )


    detections = (
        roi.get_objects_typed(
            hailo.HAILO_DETECTION
        )
    )


    largest_face = None
    largest_face_area = 0.0


    for detection in detections:

        if (
            detection.get_label()
            != "face"
        ):
            continue


        confidence = (
            detection.get_confidence()
        )


        if confidence < FACE_MIN_CONF:
            continue


        bbox = (
            detection.get_bbox()
        )


        area = (
            bbox.width()
            * bbox.height()
        )


        if area > largest_face_area:

            largest_face_area = area
            largest_face = detection


    if largest_face is not None:

        bbox = (
            largest_face.get_bbox()
        )


        cx = (
            bbox.xmin()
            + bbox.width() * 0.50
        )


        cy = (
            bbox.ymin()
            + bbox.height() * 0.50
        )


        confidence = (
            largest_face.get_confidence()
        )


        with state_lock:

            latest_face_x = cx
            latest_face_y = cy
            latest_face_conf = confidence
            latest_face_time = (
                time.monotonic()
            )


    return


# ==========================================================
# MAIN
# ==========================================================

def main():

    global running

    control_thread = None


    hailo_logger.info(
        "Starting Hailo Smooth-Q3 face tracker."
    )


    try:

        connect_q3()


        if not manual_centering():

            print(
                "Cancelled."
            )

            return


        control_thread = (
            threading.Thread(
                target=control_loop,
                daemon=True,
            )
        )


        control_thread.start()


        user_data = (
            user_app_callback_class()
        )


        app = (
            GStreamerFaceTrackApp(
                app_callback,
                user_data,
            )
        )


        app.run()


    except KeyboardInterrupt:

        print(
            "\nStopping..."
        )


    finally:

        running = False


        if control_thread is not None:

            control_thread.join(
                timeout=1.0
            )


        if q3 is not None:

            try:

                q3.disconnect()

            except:
                pass


        print(
            "Q3 disconnected."
        )


if __name__ == "__main__":
    main()
