#include "WillyIK.h"
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>

// Hexapod setup
Hexapod quadruped(43, 60, 104);

// Movement parameters
double stepLength = 50;
double stepHeight = 20;
double stepDuration = 1.0;

// Structures for data
struct XboxData
{
  bool button_a;
  bool button_b;
  bool button_y;
  bool button_x;
};

XboxData xboxData = {false};
volatile bool isMoving = false;

// Web server setup
WebServer server(80);

// Peer setup for ESP-NOW
uint8_t peerAddress[] = {0xcc, 0xdb, 0xa7, 0x62, 0xe3, 0x8c}; // Replace with actual MAC
esp_now_peer_info_t peerInfo;

// Task handles
TaskHandle_t taskWebServerHandle;
TaskHandle_t taskRobotControlHandle;

TaskHandle_t taskBalanceHandle;

void balanceTask(void *parameter)
{
  while (true)
  {
    if (!isMoving)
    {
      quadruped.balanceBody(); // Run the Balance function
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100ms
  }
}

// Function to handle movement logic
void performMovement(const String &action = "")
{
  isMoving = true;
  // Serial.println("Performing movement..." + action);
  // send is moving to the other esp as handshake
  esp_now_send(peerAddress, (uint8_t *)&isMoving, sizeof(isMoving));
  if (action == "FORWARD")
  {
    quadruped.walkWaveGait(stepLength, stepHeight, stepDuration, WALK, WALK_FORWARD);
  }
  else if (action == "BACKWARD")
  {
    quadruped.walkWaveGait(stepLength, stepHeight, stepDuration, WALK, WALK_BACKWARDS);
  }
  else if (action == "LEFT")
  {
    quadruped.walkWaveGait(stepLength, stepHeight, stepDuration, ROTATE, ROTATE_LEFT);
  }
  else if (action == "RIGHT")
  {
    quadruped.walkWaveGait(stepLength, stepHeight, stepDuration, ROTATE, ROTATE_RIGHT);
  }
  else if (action == "BALANCE")
  {
    quadruped.balanceBody();
  }
  else if (action == "SIT")
  {
    quadruped.sittingAction();
  }
  else if (action == "STAND")
  {
    quadruped.initializeStance();
  }
  else if (action == "WAVE")
  {
    quadruped.waveAction();
  }
  // delay(200); // Simulate action duration
  isMoving = false;
  esp_now_send(peerAddress, (uint8_t *)&isMoving, sizeof(isMoving));
}

void OnDataReceived(const uint8_t *mac_addr, const uint8_t *data, int len)
{
  if (len == sizeof(XboxData))
  {
    memcpy(&xboxData, data, sizeof(XboxData));
    Serial.printf("Received data from %02x:%02x:%02x:%02x:%02x:%02x\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    bool moving;
    moving = isMoving;
    if (!moving)
    {
      static String action;
      if (xboxData.button_a)
        action = "FORWARD";
      else if (xboxData.button_b)
        action = "BACKWARD";
      else if (xboxData.button_x)
        action = "LEFT";
      else if (xboxData.button_y)
        action = "RIGHT";
      Serial.println(action);
      if (action.length() > 0)
      {
        xTaskNotify(taskRobotControlHandle, (uint32_t)&action, eSetValueWithOverwrite);
      }
    }
  }
}

// Web server task for Core 1
void webServerTask(void *parameter)
{
  // Web server routes
  server.on("/", []()
            { server.send(200, "text/html", R"rawliteral(
 <!DOCTYPE html>
<html>
<head>
    <title>Robot Dashboard</title>
    <style>
        body {
            font-family: 'Arial', sans-serif;
            margin: 0;
            padding: 0;
            background: linear-gradient(120deg, #f4f4f9, #e0e7ff);
            text-align: center;
        }
        h1 {
            margin: 20px 0;
            color: #333;
        }
        #data-container {
            display: flex;
            justify-content: center;
            gap: 20px;
            margin: 20px auto;
        }
        .card {
            width: 150px;
            height: 100px;
            background: #fff;
            border-radius: 10px;
            box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            transition: transform 0.2s;
        }
        .card:hover {
            transform: scale(1.05);
        }
        .value {
            font-size: 24px;
            font-weight: bold;
            color: #0078d7;
        }
        .controls-container {
            margin: 30px auto;
        }
        .button, .dropdown {
            margin: 10px;
            padding: 15px 30px;
            font-size: 18px;
            background-color: #0078d7;
            color: #fff;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
            transition: background-color 0.3s, transform 0.2s;
        }
        .button:hover, .dropdown:hover {
            background-color: #005bb5;
            transform: translateY(-2px);
        }
        .slider-container {
            margin: 20px auto;
        }
        .slider-label {
            margin-right: 10px;
            font-weight: bold;
        }
        .animation-container {
            margin-top: 30px;
            width: 200px;
            height: 200px;
            background: #fff;
            border-radius: 50%;
            box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
            position: relative;
            transform-style: preserve-3d;
            perspective: 500px;
            display: inline-block;
        }
        .robot-representation {
            width: 100%;
            height: 100%;
            background: linear-gradient(to bottom, #0078d7, #005bb5);
            border-radius: 50%;
            position: absolute;
            top: 0;
            left: 0;
            transition: transform 0.3s ease-in-out;
        }

        /* Mobile responsive adjustments */
        @media (max-width: 600px) {
            .card {
                width: 120px;
                height: 80px;
            }
            .controls-container {
                margin: 15px auto;
            }
            .slider-label {
                font-size: 14px;
            }
            .slider-container {
                width: 100%;
            }
            .button, .dropdown {
                width: 100%;
                font-size: 16px;
                padding: 12px 20px;
            }
        }

        /* Success popup */
        #successPopup {
            position: fixed;
            bottom: 20px;
            left: 50%;
            transform: translateX(-50%);
            background: #4caf50;
            color: white;
            padding: 10px 20px;
            border-radius: 5px;
            display: none;
            opacity: 0;
            transition: opacity 0.5s ease-out;
        }

        #successPopup.show {
            display: block;
            opacity: 1;
        }
    </style>
    <script>
        let pitch = 0, roll = 0;
        let timeout;

        function fetchData() {
            fetch('/data/receive')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('roll').textContent = data.roll.toFixed(2);
                    document.getElementById('pitch').textContent = data.pitch.toFixed(2);
                    document.getElementById('cputemp').textContent = data.cputemp.toFixed(2);
                    document.getElementById('bodyTemp').textContent = data.bodyTemp.toFixed(2);
                    pitch = data.pitch;
                    roll = data.roll;
                    updateAnimation();
                })
                .catch(console.error);
        }

        function sendData() {
            const pitch = document.getElementById('pitch-adjust').value;
            const roll = document.getElementById('roll-adjust').value;
            const stepLength = document.getElementById('step-length').value;
            const stepHeight = document.getElementById('step-height').value;
            const speed = document.getElementById('speed-slider').value;
            const height = document.getElementById('height-slider').value;
            fetch('/data/send?pitch=' + pitch + '&roll=' + roll + '&stepLength=' + stepLength + '&stepHeight=' + stepHeight+ '&height=' + height+ '&speed=' + speed)
                .then(response => response.text())
                .then(() => showSuccessPopup())
                .catch(console.error);
        }

        function showSuccessPopup() {
            const popup = document.getElementById('successPopup');
            
            // Show the popup
            popup.classList.add('show');
            
            // After 3 seconds, hide the popup
            setTimeout(function() {
                popup.classList.remove('show');
            }, 3000); // The popup will disappear after 3 seconds
        }

        function updateAnimation() {
            const robot = document.querySelector('.robot-representation');
            // Applying pitch (rotateX) and roll (rotateY) to the robot
            robot.style.transform = `rotateX(${pitch}deg) rotateY(${roll}deg)`;
        }

        function sendAction(action) {
            fetch(`/action?move=${action}`)
                .then(response => response.text())
                .then(() => showSuccessPopup())
                .catch(console.error);
        }

        function resetAll() {
            document.getElementById('pitch-adjust').value = 0;
            document.getElementById('roll-adjust').value = 0;
            document.getElementById('speed-slider').value = 0.05;
            document.getElementById('step-length').value = 50;
            document.getElementById('step-height').value = 20;
            document.getElementById('pitch-adjust-value').textContent = 0;
            document.getElementById('roll-adjust-value').textContent = 0;
            document.getElementById('speed-value').textContent = 0.05;
            document.getElementById('height-value').textContent = -35;
            document.getElementById('step-length-value').textContent = 50;
            document.getElementById('step-height-value').textContent = 20;
            document.getElementById('base-height').value = -35;
            pitch = 0;
            roll = 0;
            updateAnimation();
            sendData();
            showSuccessPopup();
        }

        function updateSliderValue(id, value) {
            document.getElementById(id).textContent = value;

            // Clear previous timeout and delay sending data
            clearTimeout(timeout);
            timeout = setTimeout(() => {
                sendData();
            }, 500);  // 500ms delay before sending data
        }

        setInterval(fetchData, 1000);
    </script>
</head>
<body>
    <h1>Robot Dashboard</h1>
    <div id="data-container">
        <div class="card">
            <div>Roll</div>
            <div class="value" id="roll">--</div>
        </div>
        <div class="card">
            <div>Pitch</div>
            <div class="value" id="pitch">--</div>
        </div>
        <div class="card">
            <div>CPU Temp</div>
            <div class="value" id="cputemp">--</div>
        </div>
        <div class="card">
            <div>Body Temp</div>
            <div class="value" id="bodyTemp">--</div>
        </div>
    </div>
    <div class="controls-container">
        <select class="dropdown" id="action-dropdown">
            <option value="SIT">Sit</option>
            <option value="STAND">Stand</option>
            <option value="WAVE">Wave</option>
        </select>
        <button class="button" onclick="sendAction(document.getElementById('action-dropdown').value)">Execute Action</button>
    </div>
    <div class="controls-container">
        <button class="button" onclick="sendAction('FORWARD')">Move Forward</button>
        <button class="button" onclick="sendAction('BACKWARD')">Move Backward</button>
        <button class="button" onclick="sendAction('LEFT')">Turn Left</button>
        <button class="button" onclick="sendAction('RIGHT')">Turn Right</button>
    </div>
    <div class="controls-container">
        <button class="button" onclick="sendAction('BALANCE')">Balance</button>
        <button class="button" onclick="resetAll()">Reset All</button>
    </div>
    <div class="controls-container">
        <label class="slider-label">Pitch Adjust:</label>
        <input id="pitch-adjust" type="range" min="-30" max="30" value="0" oninput="updateSliderValue('pitch-adjust-value', this.value)" />
        <span id="pitch-adjust-value">0</span>
    </div>
    <div class="controls-container">
        <label class="slider-label">Roll Adjust:</label>
        <input id="roll-adjust" type="range" min="-30" max="30" value="0" oninput="updateSliderValue('roll-adjust-value', this.value)" />
        <span id="roll-adjust-value">0</span>
    </div>
    <div class="controls-container">
        <label class="slider-label">Speed:</label>
        <input id="speed-slider" type="range" min="0.01" max="0.5" step="0.01" value="0.05" oninput="updateSliderValue('speed-value', this.value)" />
        <span id="speed-value">0.05</span>
    </div>
    <div class="controls-container">
      <label class="slider-label">Height:</label>
      <input id="height-slider" type="range" min="-40" max="0" value="-30" oninput="updateSliderValue('height-value', this.value)" />
      <span id="height-value">-30</span>
    </div>
    <div class="controls-container">
        <label class="slider-label">Step Length:</label>
        <input id="step-length" type="range" min="5" max="70" value="50" oninput="updateSliderValue('step-length-value', this.value)" />
        <span id="step-length-value">50</span>
    </div>
    <div class="controls-container">
        <label class="slider-label">Step Height:</label>
        <input id="step-height" type="range" min="0" max="30" value="20" oninput="updateSliderValue('step-height-value', this.value)" />
        <span id="step-height-value">20</span>
    </div>
    <div class="animation-container">
        <div class="robot-representation"></div>
    </div>

    <!-- Success Popup -->
    <div id="successPopup">Values Set Successfully!</div>
</body>
</html>

)rawliteral"); });

  server.on("/data/receive", []()
            {
        String json = "{\"roll\":" + String(quadruped.getRoll()) + ",\"pitch\":" + String(quadruped.getPitch()) +",\"cputemp\":" +String(temperatureRead())+ ",\"bodyTemp\":"+String(quadruped.getTemp())+"}";
        server.send(200, "application/json", json); });
  server.on("/data/send", []()
            {
        if (server.hasArg("pitch") && server.hasArg("roll") && server.hasArg("stepLength") && server.hasArg("stepHeight")&&server.hasArg("height"))
        {
            float pitch = server.arg("pitch").toFloat();
            float roll = server.arg("roll").toFloat();
            stepLength = server.arg("stepLength").toFloat();
            stepHeight = server.arg("stepHeight").toFloat();
            stepDuration = server.arg("stepDuration").toFloat();
            double speed = server.arg("speed").toFloat();
            quadruped.setTimescale(speed);
            quadruped.setHeight(server.arg("height").toFloat());
            quadruped.SetPidTargets(pitch,roll);
            Serial.println("PID targets set and are: "+String(pitch)+" "+String(roll));
            // server.send(200, "text/plain", "Data received.");
          Serial.println("height set to: "+String(server.arg("height").toFloat()));
          Serial.println("speed set to: "+String(speed));
          Serial.println("stepLength set to: "+String(stepLength));
          Serial.println("stepHeight set to: "+String(stepHeight));
        
        } 
        else
        {
            server.send(400, "text/plain", "Bad Request");
        } });
  server.on("/action", []()
            {
        if (server.hasArg("move")) {
            String move = server.arg("move");
            if (!isMoving) {
                
                xTaskNotify(taskRobotControlHandle, (uint32_t)&move, eSetValueWithOverwrite);
                server.send(200, "text/plain", "Action " + move + " performed.");
            } else {
                server.send(200, "text/plain", "Robot is already moving.");
            }
        } else {
            server.send(400, "text/plain", "Bad Request");
        } });

  server.begin();

  // Handle client requests
  while (true)
  {
    server.handleClient();
    delay(10); // Yield to other tasks
  }
}

// Robot control task for Core 0
void robotControlTask(void *parameter)
{
  uint32_t notificationValue;
  while (true)
  {
    xTaskNotifyWait(0, ULONG_MAX, &notificationValue, portMAX_DELAY);
    String *action = (String *)notificationValue;
    if (action != nullptr)
    {
      performMovement(*action);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  while (!Serial)
    ;

  // Initialize Wi-Fi
  WiFi.mode(WIFI_AP_STA);
  // WiFi.begin("108B", "108BCameraSmecherii$"); // Replace with your Wi-Fi credentials
  Serial.println("Setting up AP...");
  WiFi.softAP("internalNET", "jamal1234"); // Start the AP with SSID and password
  Serial.println("AP Started.");
  WiFi.begin("SpiderBot", "12345678");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Connected to Wi-Fi: " + WiFi.localIP().toString());

  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP().toString());
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataReceived);
  esp_now_register_send_cb([](const uint8_t *mac, esp_now_send_status_t status)
                           {
    if (status == ESP_OK)
    {
      Serial.println("Data sent successfully");
    }
    else
    {
      Serial.println("Error sending data");
    } });
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;

  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  // Initialize robot
  quadruped.InitializeRobotControllers();

  // Create tasks for each core
  xTaskCreatePinnedToCore(webServerTask, "WebServerTask", 4096, NULL, 1, &taskWebServerHandle, 0);
  xTaskCreatePinnedToCore(robotControlTask, "RobotControlTask", 4096, NULL, 1, &taskRobotControlHandle, 1);
  xTaskCreatePinnedToCore(balanceTask, "BalanceTask", 4096, NULL, 1, &taskBalanceHandle, 1);
}
void loop()
{
  // Main loop can be used for debugging or other non-blocking tasks
}
