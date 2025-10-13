# Robot Arm API Examples

## Overview
This document provides examples for using the Robot Arm API. The API allows you to control the robot arm programmatically using HTTP requests.

## Base URL
```
http://[ESP32_IP_ADDRESS]/api/
```

Replace `[ESP32_IP_ADDRESS]` with your ESP32's IP address (e.g., `192.168.1.100`)

## Authentication
No authentication is required for this version of the API.

## Content-Type
- For POST requests, use: `application/x-www-form-urlencoded`
- Responses are in: `application/json`

## Response Format
All API responses follow this standard format:
```json
{
  "status": "success|error",
  "message": "Human readable message",
  "data": { /* Response data */ },
  "timestamp": 1234567890
}
```

## API Endpoints

### 1. Move Robot to XYZ Coordinates
Move the robot arm to specific X, Y, Z coordinates.

**Endpoint:** `POST /api/move`

**Parameters:**
- `x` (required): X coordinate (integer)
- `y` (required): Y coordinate (integer)  
- `z` (required): Z coordinate (integer)
- `speed` (optional): Movement speed (integer)
- `acceleration` (optional): Movement acceleration (integer)

**Examples:**

**cURL:**
```bash
curl -X POST http://192.168.1.100/api/move \
  -d "x=100&y=150&z=200&speed=1000&acceleration=500"
```

**JavaScript (Fetch):**
```javascript
fetch('http://192.168.1.100/api/move', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/x-www-form-urlencoded',
  },
  body: 'x=100&y=150&z=200&speed=1000&acceleration=500'
})
.then(response => response.json())
.then(data => console.log(data));
```

**Python (requests):**
```python
import requests

response = requests.post('http://192.168.1.100/api/move', 
                        data={
                            'x': 100, 
                            'y': 150, 
                            'z': 200,
                            'speed': 1000,
                            'acceleration': 500
                        })
print(response.json())
```

**Arduino/ESP32:**
```cpp
#include <HTTPClient.h>

void moveRobot() {
  HTTPClient http;
  http.begin("http://192.168.1.100/api/move");
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  
  String postData = "x=100&y=150&z=200&speed=1000&acceleration=500";
  int httpResponseCode = http.POST(postData);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println(response);
  }
  http.end();
}
```

### 2. Move Robot Joints
Move robot joints to specific angles.

**Endpoint:** `POST /api/joint`

**Parameters:**
- `theta1` (required): Joint 1 angle in degrees (integer)
- `theta2` (required): Joint 2 angle in degrees (integer)
- `theta3` (required): Joint 3 angle in degrees (integer)
- `speed` (optional): Movement speed (integer)
- `acceleration` (optional): Movement acceleration (integer)

**Examples:**

**cURL:**
```bash
curl -X POST http://192.168.1.100/api/joint \
  -d "theta1=45&theta2=30&theta3=60&speed=800"
```

**Python:**
```python
import requests

response = requests.post('http://192.168.1.100/api/joint', 
                        data={
                            'theta1': 45, 
                            'theta2': 30, 
                            'theta3': 60,
                            'speed': 800
                        })
print(response.json())
```

### 3. Control Gripper
Control the gripper position.

**Endpoint:** `POST /api/gripper`

**Parameters:**
- `position` (required): Gripper position (0-180, integer)

**Examples:**

**cURL:**
```bash
# Open gripper
curl -X POST http://192.168.1.100/api/gripper -d "position=180"

# Close gripper
curl -X POST http://192.168.1.100/api/gripper -d "position=0"

# Half open
curl -X POST http://192.168.1.100/api/gripper -d "position=90"
```

**JavaScript:**
```javascript
// Function to control gripper
function controlGripper(position) {
  fetch('http://192.168.1.100/api/gripper', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/x-www-form-urlencoded',
    },
    body: `position=${position}`
  })
  .then(response => response.json())
  .then(data => console.log(data));
}

// Usage
controlGripper(0);   // Close
controlGripper(180); // Open
```

### 4. Get Robot Status
Get current robot status and position information.

**Endpoint:** `GET /api/status`

**Examples:**

**cURL:**
```bash
curl -X GET http://192.168.1.100/api/status
```

**JavaScript:**
```javascript
fetch('http://192.168.1.100/api/status')
  .then(response => response.json())
  .then(data => {
    console.log('Current Position:', data.data.current_position);
    console.log('Gripper Position:', data.data.gripper_position);
    console.log('Queue Size:', data.data.queue_size);
  });
```

**Python:**
```python
import requests

response = requests.get('http://192.168.1.100/api/status')
status_data = response.json()

print(f"Current X,Y,Z: {status_data['data']['last_coordinates']}")
print(f"Joint Angles: {status_data['data']['joint_angles']}")
print(f"Gripper: {status_data['data']['gripper_position']}")
print(f"Queue Size: {status_data['data']['queue_size']}")
```

### 5. Execute G-code
Execute G-code commands for complex sequences.

**Endpoint:** `POST /api/gcode`

**Parameters:**
- `gcode` (required): G-code commands (string, multiline supported)

**Supported G-codes:**
- `X###Y###Z###` - Move to coordinates
- `GPON` - Turn gripper on (close)
- `GPOFF` - Turn gripper off (open)
- `DELAY###` - Delay in milliseconds

**Examples:**

**cURL:**
```bash
curl -X POST http://192.168.1.100/api/gcode \
  -d "gcode=X100Y150Z200
GPON
DELAY1000
X200Y200Z100
GPOFF"
```

**Python:**
```python
import requests

gcode_sequence = """X100Y150Z200
GPON
DELAY1000
X50Y50Z50
DELAY500
GPOFF
X0Y0Z0"""

response = requests.post('http://192.168.1.100/api/gcode', 
                        data={'gcode': gcode_sequence})
print(response.json())
```

**JavaScript:**
```javascript
const gcodeSequence = `X100Y150Z200
GPON
DELAY1000
X200Y200Z100
GPOFF`;

fetch('http://192.168.1.100/api/gcode', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/x-www-form-urlencoded',
  },
  body: `gcode=${encodeURIComponent(gcodeSequence)}`
})
.then(response => response.json())
.then(data => console.log(data));
```

### 6. Home Robot
Move robot to home position.

**Endpoint:** `POST /api/home`

**Examples:**

**cURL:**
```bash
curl -X POST http://192.168.1.100/api/home
```

**Python:**
```python
import requests

response = requests.post('http://192.168.1.100/api/home')
print(response.json())
```

### 7. Zero/Calibrate Robot
Calibrate the robot using limit switches.

**Endpoint:** `POST /api/zero`

**Examples:**

**cURL:**
```bash
curl -X POST http://192.168.1.100/api/zero
```

**Python:**
```python
import requests

response = requests.post('http://192.168.1.100/api/zero')
print(response.json())
```

## Complete Example Applications

### 1. Simple Pick and Place Sequence

**Python:**
```python
import requests
import time

robot_ip = "192.168.1.100"
base_url = f"http://{robot_ip}/api"

def robot_request(endpoint, data=None):
    if data:
        response = requests.post(f"{base_url}/{endpoint}", data=data)
    else:
        response = requests.post(f"{base_url}/{endpoint}")
    print(f"{endpoint}: {response.json()['message']}")
    return response.json()

# Pick and place sequence
print("Starting pick and place sequence...")

# 1. Move to pickup position
robot_request("move", {"x": 100, "y": 100, "z": 50})
time.sleep(2)

# 2. Lower to pick up object
robot_request("move", {"x": 100, "y": 100, "z": 10})
time.sleep(1)

# 3. Close gripper
robot_request("gripper", {"position": 0})
time.sleep(1)

# 4. Lift object
robot_request("move", {"x": 100, "y": 100, "z": 50})
time.sleep(2)

# 5. Move to drop position
robot_request("move", {"x": 200, "y": 200, "z": 50})
time.sleep(2)

# 6. Lower to drop position
robot_request("move", {"x": 200, "y": 200, "z": 10})
time.sleep(1)

# 7. Open gripper
robot_request("gripper", {"position": 180})
time.sleep(1)

# 8. Return to safe position
robot_request("move", {"x": 200, "y": 200, "z": 50})
time.sleep(1)

# 9. Go home
robot_request("home")

print("Pick and place sequence completed!")
```

### 2. Web Interface Example

**HTML + JavaScript:**
```html
<!DOCTYPE html>
<html>
<head>
    <title>Robot Arm Control</title>
</head>
<body>
    <h1>Robot Arm Control Panel</h1>
    
    <div>
        <h3>Move to Position</h3>
        <input type="number" id="x" placeholder="X" value="100">
        <input type="number" id="y" placeholder="Y" value="100">
        <input type="number" id="z" placeholder="Z" value="100">
        <button onclick="moveRobot()">Move</button>
    </div>
    
    <div>
        <h3>Gripper Control</h3>
        <button onclick="controlGripper(0)">Close</button>
        <button onclick="controlGripper(90)">Half</button>
        <button onclick="controlGripper(180)">Open</button>
    </div>
    
    <div>
        <h3>Quick Actions</h3>
        <button onclick="homeRobot()">Home</button>
        <button onclick="zeroRobot()">Zero/Calibrate</button>
        <button onclick="getStatus()">Get Status</button>
    </div>
    
    <div id="status"></div>
    
    <script>
        const robotIP = "192.168.1.100"; // Change to your robot's IP
        const apiBase = `http://${robotIP}/api`;
        
        function moveRobot() {
            const x = document.getElementById('x').value;
            const y = document.getElementById('y').value;
            const z = document.getElementById('z').value;
            
            fetch(`${apiBase}/move`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/x-www-form-urlencoded',
                },
                body: `x=${x}&y=${y}&z=${z}`
            })
            .then(response => response.json())
            .then(data => {
                document.getElementById('status').innerHTML = 
                    `<strong>Move:</strong> ${data.message}`;
            });
        }
        
        function controlGripper(position) {
            fetch(`${apiBase}/gripper`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/x-www-form-urlencoded',
                },
                body: `position=${position}`
            })
            .then(response => response.json())
            .then(data => {
                document.getElementById('status').innerHTML = 
                    `<strong>Gripper:</strong> ${data.message}`;
            });
        }
        
        function homeRobot() {
            fetch(`${apiBase}/home`, {method: 'POST'})
            .then(response => response.json())
            .then(data => {
                document.getElementById('status').innerHTML = 
                    `<strong>Home:</strong> ${data.message}`;
            });
        }
        
        function zeroRobot() {
            fetch(`${apiBase}/zero`, {method: 'POST'})
            .then(response => response.json())
            .then(data => {
                document.getElementById('status').innerHTML = 
                    `<strong>Zero:</strong> ${data.message}`;
            });
        }
        
        function getStatus() {
            fetch(`${apiBase}/status`)
            .then(response => response.json())
            .then(data => {
                const status = data.data;
                document.getElementById('status').innerHTML = `
                    <h4>Robot Status</h4>
                    <p><strong>Position:</strong> X:${status.last_coordinates.x}, Y:${status.last_coordinates.y}, Z:${status.last_coordinates.z}</p>
                    <p><strong>Gripper:</strong> ${status.gripper_position}°</p>
                    <p><strong>Queue:</strong> ${status.queue_size} commands</p>
                    <p><strong>WiFi:</strong> ${status.wifi_connected ? 'Connected' : 'Disconnected'}</p>
                `;
            });
        }
    </script>
</body>
</html>
```

## Error Handling

Always check the response status:

```python
import requests

try:
    response = requests.post('http://192.168.1.100/api/move', 
                           data={'x': 100, 'y': 100, 'z': 100})
    
    if response.status_code == 200:
        data = response.json()
        if data['status'] == 'success':
            print(f"Success: {data['message']}")
        else:
            print(f"API Error: {data['message']}")
    else:
        print(f"HTTP Error: {response.status_code}")
        
except requests.exceptions.RequestException as e:
    print(f"Connection Error: {e}")
```

## Rate Limiting

Be mindful of command queue size. Check status before sending many commands:

```python
def safe_command(endpoint, data):
    # Check queue size first
    status = requests.get('http://192.168.1.100/api/status').json()
    if status['data']['queue_size'] > 10:
        print("Queue full, waiting...")
        time.sleep(1)
        return safe_command(endpoint, data)
    
    # Send command
    return requests.post(f'http://192.168.1.100/api/{endpoint}', data=data)
```

## Documentation
For complete API documentation with interactive testing, visit:
`http://[ESP32_IP_ADDRESS]/api/docs`