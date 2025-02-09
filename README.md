# ARC Authentication Framework

ARC (Advanced Robotic Control) is a multi-layered, multi-sensor authentication protocol designed for robotic systems operating under ROS2. The framework provides real-time, multi-factor authentication by fusing diverse authentication modalities into a single confidence score.

## Overview

ARC determines user identity and access levels by combining multiple authentication factors:
- Facial Recognition
- Voice Analysis (text-dependent and independent)
- Fingerprint Scanning
- RFID with anti-cloning measures
- Extensible for additional biometrics

The system maps authentication confidence scores to eight defined access levels, ranging from basic non-sensitive functions to full administrative control.

## Key Features

- **Real-time Authentication**: < 100ms latency under standard conditions
- **Multi-factor Fusion**: Combines multiple authentication modalities
- **Flexible Access Levels**: 8 configurable access tiers
- **Robust Security**:
  - Exponential backoff for failed attempts
  - Multi-layer reauthentication
  - RFID anti-cloning protection
- **Modular Architecture**: Easily extensible sensor plugin framework
- **ROS2 Integration**: Native ROS2 nodes with DDS-Security

## Architecture

### Core Components

1. **Authentication Core**
   - Confidence score calculation
   - Access level determination
   - Retry and lockout management

2. **Sensor Plugin Framework**
   - Abstract sensor interface
   - Modular sensor implementations
   - Real-time data processing

3. **ROS2 Integration**
   - Secure communication (SROS2)
   - Service-based authentication
   - Role-based access control

### Authentication Sensors

Currently implemented sensors:

1. **Facial Recognition Sensor**
   - OpenCV-based face detection
   - Face encoding and matching
   - Real-time video processing

2. **Voice Recognition Sensor**
   - Text-dependent verification
   - Voice biometric analysis
   - Speech-to-text verification

3. **Future Sensors** (Planned)
   - Fingerprint scanning
   - Secure RFID
   - Additional biometrics

## Access Levels

| Level | Name | Description |
|-------|------|-------------|
| 0 | NONE | No access |
| 1 | BASIC | Non-sensitive functions |
| 2 | LIMITED | Basic operational access |
| 3 | STANDARD | Normal operational access |
| 4 | ENHANCED | Enhanced operational access |
| 5 | PRIVILEGED | Privileged system functions |
| 6 | ADMIN | Administrative access |
| 7 | ROOT | Root-level system access |
| 8 | FULL | Complete system control |

## Installation

### Prerequisites
- Python 3.8 or higher
- ROS2 Humble or newer
- OpenCV 4.x
- CUDA-capable GPU (recommended)

### Basic Installation

```
bash
git clone https://github.com/a-bissell/ARC-framework
pip install -r requirements.txt
```

### ROS2 Installation

```
bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
``` 
## Usage
To use the ARC Authentication Framework, follow these steps:

1. **Source ROS2 Environment**
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

2. **Launch the Authentication Node**
   ```bash
   ros2 run arc_auth authentication_node
   ```

3. **Enroll a New User**
   ```bash
   # Enroll a user with facial recognition
   ros2 service call /enroll_user arc_auth_interfaces/srv/EnrollUser "{user_id: 'user123', factor_type: 'facial'}"
   ```

4. **Authenticate a User**
   ```bash
   # Request authentication for a user
   ros2 topic pub /authenticate_user arc_auth_interfaces/msg/AuthenticationRequest "{user_id: 'user123'}"
   ```

5. **Monitor Authentication Status**
   ```bash
   # Subscribe to authentication results
   ros2 topic echo /authentication_response
   ```

The authentication node will process the request using configured sensors and return an AccessLevel based on the authentication confidence. Multiple authentication factors can be combined for higher security levels.

Example authentication flow:
1. Start the authentication node
2. Enroll user biometric data
3. Request authentication when access is needed
4. System will activate sensors and verify identity
5. Access level will be granted based on authentication confidence

For development and testing, you can use the included test scripts.

## Running Tests

To run the test suite for the ARC Authentication Framework:

1. **Run Unit Tests**
   ```bash
   colcon test --packages-select arc_auth
   ```

2. **View Test Results**
   ```bash
   colcon test-result --verbose
   ```

3. **Run Integration Tests**
   ```bash
   # Launch test environment
   ros2 launch arc_auth test_launch.py
   
   # In another terminal, run integration tests
   python3 -m pytest src/arc_auth/test/integration/
   ```

The test suite includes:
- Unit tests for core authentication logic
- Sensor simulation tests
- Integration tests for ROS2 nodes and services
- End-to-end authentication workflow tests

For development, you can run specific test files:
pytest src/arc_auth/tests/test_voice_sensor.py
pytest src/arc_auth/tests/test_facial_sensor.py


