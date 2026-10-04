# Smooth-Q3 Face Tracking

Raspberry Pi face/person tracking project that controls a Zhiyun Smooth-Q3 gimbal over Bluetooth Low Energy.

## Hardware

- Raspberry Pi
- Hailo AI accelerator
- Zhiyun Smooth-Q3 gimbal
- Camera connected to the Raspberry Pi

## Software

The main program is:

```text
person_track_face_q3.py
```

The program detects and tracks a person/face and sends BLE commands to the Smooth-Q3 gimbal to keep the subject centered.

## Smooth-Q3 BLE Information

- Device address: `D5:4D:56:D7:F2:DA`
- Service: `FEE9`
- Write characteristic: `d44bc439-abfd-45a2-b575-925416129600`
- Write handle: `51`

The program uses `bluepy` to communicate with the gimbal.

## Running

Activate the Hailo environment first:

```bash
source ~/hailo-apps/venv_hailo_apps/bin/activate
```

Then run:

```bash
python3 ~/Gimbal-Tracking/person_track_face_q3.py
```

The program centers the gimbal at startup before beginning tracking.

## Status

Working face/person tracking with automatic gimbal centering at startup.
