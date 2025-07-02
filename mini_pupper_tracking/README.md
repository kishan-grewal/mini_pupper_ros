## Introduction 
The tracking package was developed during the 2025 Global Internship Programme by HKSTP.

It is only supported with the Stanford Controller and not the CHAMP Controller.

Launch file not yet supported.

## 1. Getting the ONNX ...

## 2. Setup ...

## 3. Quick Start

### **Mini Pupper**
```sh
# Terminal 1 (SSH)
. ~/ros2_ws_new/install/setup.bash # Use setup.zsh if you use zsh instead of bash
ros2 launch mini_pupper_bringup bringup_with_stanford_controller.launch.py
```

### **PC**
```sh
# Terminal 2
source ~/ros2_ws/install/setup.bash
ros2 launch stanford_controller twist_to_command_converter.launch.py
```

```sh
# Terminal 3
source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_tracking tracking.launch.py
```