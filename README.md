# Dobot Qt Joint Control

Python Qt interface for monitoring and controlling Dobot robot joint angles over TCP/IP.

## Files

- `qt_joint_control.py`: Qt GUI. It reads `q_actual` from the feedback port and controls the robot with `JointMovJ` or optional `ServoJ`.
- `dobot_api.py`: Dobot TCP/IP Python SDK wrapper.
- `files/alarm_controller.json`, `files/alarm_servo.json`: Alarm metadata used by the SDK.

## Requirements

- Python 3.8+
- Robot controller in TCP/IP secondary development mode
- Local computer on the robot network, usually `192.168.5.x`

Install dependencies:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
python3 qt_joint_control.py
```

Default connection values:

- Robot IP: `192.168.5.1`
- Dashboard port: `29999`
- Move port: `30003`
- Feedback port: `30004`

## Usage Notes

1. Start with a low speed factor, for example `20`.
2. Click `Connect`.
3. Wait for the first feedback frame. The slider target values are synchronized to the actual joint angles automatically.
4. Use `Enable` only after checking that the robot workspace is clear.
5. Drag a slider. By default, releasing the slider sends one `JointMovJ` command with all six target joint angles.
6. `Realtime ServoJ while dragging` sends continuous `ServoJ` commands while dragging. Use it only after verifying the robot behavior at low speed.

The GUI displays actual joint angles from `q_actual`, robot mode, speed scaling, enable state, running state, and error state.
