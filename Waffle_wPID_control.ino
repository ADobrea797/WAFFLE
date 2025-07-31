#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "sensirion-lf.h"
#include <PID_v1.h>  // Include the Arduino PID library

// Network credentials
const char* ssid = "XXX"; //Replace with network name
const char* password = "XXX";//Replace with network password

// Web server
AsyncWebServer server(80);

// Shift register pins
const int dataPin = 35;
const int latchPin = 36;
const int slaveSelectPin = 33;
const int outputEnablePin = 34;
const int valveEnablePin = 6; 
const int flowcellEnablePin = 16; 
unsigned long lastVoltageUpdateTime = 0;  // Track last update time
const unsigned long voltageUpdateInterval = 200;  // Interval for voltage updates (in ms)

const int numReadings = 10;   // Number of readings for the moving average
double flowReadings[numReadings]; // Array to store flow readings
int readIndex = 0;             // Current index in the array
double totalFlow = 0;          // Sum of the readings
double averageFlow = 0;        // Moving average of the readings

// Valve states
bool output26State = false;
bool output27State = false;
bool output28State = false;
bool output29State = false;
bool output30State = false;
bool output31State = false;
bool output32State = false;
bool output33State = false;
bool flowcellEnableState = false; 

// Pump control variables
boolean bPumpState = false;
uint8_t nPumpVoltageByte = 0x00;
uint16_t currentFrequency = 100; // Locked frequency at 100 Hz (may be changed if needed)

#define I2C_LOWDRIVER_ADRESS (0x59)

// PID control variables
double flowSetpoint = 0;
double currentFlow = 0;
double pidOutput = 0;
double kp = 5, ki = 15, kd = 1;

// Initialize the PID controller
PID myPID(&currentFlow, &pidOutput, &flowSetpoint, kp, ki, kd, DIRECT);

// Function declarations
void Lowdriver_setvoltage(uint8_t _voltage);
void Lowdriver_setfrequency(uint16_t frequency);
void Lowdriver_init();
void updateShiftRegister();

// Sensirion flow sensor settings
#define MEASURE_DELAY 100


// GUI HTML Webpage design
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP Web Server</title>
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 1.84rem;}
    p {font-size: 1.52rem;}
    body {max-width: 400px; margin:0px auto; padding-bottom: 25px;}
    .button { background-color: #4CAF50; border: none; color: white; padding: 12.8px 32px;
      text-decoration: none; font-size: 24px; margin: 1.6px; cursor: pointer; }
    .button2 { background-color: #555555; }
    .slider { -webkit-appearance: none; margin: 14px; width: 360px; height: 25px; border-radius: 5px; background: #FFD65C;
      outline: none; -webkit-transition: .2s; transition: opacity .2s;}
    .checkbox { margin-top: 20px; }
    .flow-container { color: grey; }
    .flow-value { background-color: #d3d3d3; padding: 0 5px; border-radius: 3px; }
  </style>
</head>
<body>
  <h2>ESP Web Server</h2>
  <p>Flow rate: <span id="flowValue" class="flow-value">%FLOWVALUE%</span> ml/min</p>
  <p>Flow rate Setpoint: <input type="number" id="flowSetpointInput" min="0" step="0.1" value="%FLOWSETPOINT%" onchange="updateFlowSetpoint(this)"></p>
  <p>Enable Pump: <input type="checkbox" id="pumpCheckbox" class="checkbox" onchange="togglePump(this)" %PUMPSTATE%></p>
  <p>Valve 1 - State: <span id="state26">%STATE26%</span></p>
  <p><a href="/26/%TOGGLE26%"><button class="button %BTN26%">%BTNTEXT26%</button></a></p>
  <p>Valve 2 - State: <span id="state27">%STATE27%</span></p>
  <p><a href="/27/%TOGGLE27%"><button class="button %BTN27%">%BTNTEXT27%</button></a></p>
  <p>Valve 3 - State: <span id="state28">%STATE28%</span></p>
  <p><a href="/28/%TOGGLE28%"><button class="button %BTN28%">%BTNTEXT28%</button></a></p>
  <p>Valve 4 - State: <span id="state29">%STATE29%</span></p>
  <p><a href="/29/%TOGGLE29%"><button class="button %BTN29%">%BTNTEXT29%</button></a></p>
  <p>Valve 5 - State: <span id="state30">%STATE30%</span></p>
  <p><a href="/30/%TOGGLE30%"><button class="button %BTN30%">%BTNTEXT30%</button></a></p>
  <p>Valve 6 - State: <span id="state31">%STATE31%</span></p>
  <p><a href="/31/%TOGGLE31%"><button class="button %BTN31%">%BTNTEXT31%</button></a></p>
  <p>Valve 7 - State: <span id="state32">%STATE32%</span></p>
  <p><a href="/32/%TOGGLE32%"><button class="button %BTN32%">%BTNTEXT32%</button></a></p>
  <p>Valve 8 - State: <span id="state33">%STATE33%</span></p>
  <p><a href="/33/%TOGGLE33%"><button class="button %BTN33%">%BTNTEXT33%</button></a></p>
  <p>Flowcell Enable 1 - State: <span id="stateFlowcell">%STATEFLOWCELL%</span></p>
  <p><a href="/flowcell/%TOGGLEFLOWCELL%"><button class="button %BTNFlowcell%">%BTNTEXTFLOWCELL%</button></a></p>

<script>

function updateFlowSetpoint(element) {
  var flowSetpoint = element.value;
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/setflowsetpoint?flowSetpoint=" + flowSetpoint, true);
  xhr.send();
}

function togglePump(element) {
  var pumpState = element.checked ? "on" : "off";
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/setpump?state=" + pumpState, true);
  xhr.send();
}

</script>
</body>
</html>
)rawliteral";

// Replaces placeholder with dynamic values
String processor(const String& var) {
  if (var == "FLOWVALUE") return String(currentFlow, 2);
  if (var == "FLOWSETPOINT") return String(flowSetpoint, 2);
  if (var == "PUMPSTATE") return String(bPumpState ? "checked" : "");
  
  if (var == "STATE26") return String(output26State ? "ON" : "OFF");
  if (var == "TOGGLE26") return String(output26State ? "off" : "on");
  if (var == "BTN26") return String(output26State ? "button2" : "button");
  if (var == "BTNTEXT26") return String(output26State ? "OFF" : "ON");
  if (var == "STATE27") return String(output27State ? "ON" : "OFF");
  if (var == "TOGGLE27") return String(output27State ? "off" : "on");
  if (var == "BTN27") return String(output27State ? "button2" : "button");
  if (var == "BTNTEXT27") return String(output27State ? "OFF" : "ON");
  if (var == "STATE28") return String(output28State ? "ON" : "OFF");
  if (var == "TOGGLE28") return String(output28State ? "off" : "on");
  if (var == "BTN28") return String(output28State ? "button2" : "button");
  if (var == "BTNTEXT28") return String(output28State ? "OFF" : "ON");
  if (var == "STATE29") return String(output29State ? "ON" : "OFF");
  if (var == "TOGGLE29") return String(output29State ? "off" : "on");
  if (var == "BTN29") return String(output29State ? "button2" : "button");
  if (var == "BTNTEXT29") return String(output29State ? "OFF" : "ON");
  if (var == "STATE30") return String(output30State ? "ON" : "OFF");
  if (var == "TOGGLE30") return String(output30State ? "off" : "on");
  if (var == "BTN30") return String(output30State ? "button2" : "button");
  if (var == "BTNTEXT30") return String(output30State ? "OFF" : "ON");
  if (var == "STATE31") return String(output31State ? "ON" : "OFF");
  if (var == "TOGGLE31") return String(output31State ? "off" : "on");
  if (var == "BTN31") return String(output31State ? "button2" : "button");
  if (var == "BTNTEXT31") return String(output31State ? "OFF" : "ON");
  if (var == "STATE32") return String(output32State ? "ON" : "OFF");
  if (var == "TOGGLE32") return String(output32State ? "off" : "on");
  if (var == "BTN32") return String(output32State ? "button2" : "button");
  if (var == "BTNTEXT32") return String(output32State ? "OFF" : "ON");
  if (var == "STATE33") return String(output33State ? "ON" : "OFF");
  if (var == "TOGGLE33") return String(output33State ? "off" : "on");
  if (var == "BTN33") return String(output33State ? "button2" : "button");
  if (var == "BTNTEXT33") return String(output33State ? "OFF" : "ON");

  if (var == "STATEFLOWCELL") return String(flowcellEnableState ? "ON" : "OFF"); // New placeholder
  if (var == "TOGGLEFLOWCELL") return String(flowcellEnableState ? "off" : "on"); // New placeholder
  if (var == "BTNFlowcell") return String(flowcellEnableState ? "button2" : "button"); // New placeholder
  if (var == "BTNTEXTFLOWCELL") return String(flowcellEnableState ? "OFF" : "ON"); // New placeholder
  return String();
}

void setup() {

  // Initialize array to 0
    for (int i = 0; i < numReadings; i++) {
        flowReadings[i] = 0;
    }
    totalFlow = 0;
    
  // Initialize the output variables as outputs
  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(slaveSelectPin, OUTPUT);
  pinMode(valveEnablePin, OUTPUT); 
  pinMode(outputEnablePin, OUTPUT); 
  pinMode(flowcellEnablePin, OUTPUT);  

 digitalWrite(outputEnablePin, HIGH); // Initially disable the outputs
  digitalWrite(flowcellEnablePin, LOW); // Initially disable the flowcell

  Serial.begin(115200); // initialize serial communication
  if (SLF3X.init() != 0) {
    Serial.println("Error during SLF3X init. Stopping application.");
    while (1) { 
       Serial.println("Retrying SLF3X initialization...");
       delay(1000); 
       if (SLF3X.init() == 0) break;  // Try re-initializing} // loop forever
  }
}
  SPI.begin();
  digitalWrite(slaveSelectPin, LOW);

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());



  
  // Web server handlers
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/setflowsetpoint", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("flowSetpoint")) {
      flowSetpoint = request->getParam("flowSetpoint")->value().toFloat();
      Serial.println("Flow Setpoint: " + String(flowSetpoint));
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/setpump", HTTP_GET, [](AsyncWebServerRequest *request) {
    String inputMessage = request->getParam("state")->value();
    bPumpState = (inputMessage == "on");
    if (!bPumpState) Lowdriver_setvoltage(0);  // Turn off pump if disabled
    Serial.println("Pump State: " + String(bPumpState ? "ON" : "OFF"));
    request->send(200, "text/plain", "OK");
  });


  server.on("/26/on", HTTP_GET, [](AsyncWebServerRequest *request) {
    output26State = true;
    updateShiftRegister();
    delay(1000);
    digitalWrite(valveEnablePin, HIGH);
    Serial.println("Valve 1 State: ON");
    request->redirect("/");
  });

  server.on("/26/off", HTTP_GET, [](AsyncWebServerRequest *request) {
    output26State = false;
    digitalWrite(valveEnablePin, LOW);
    Serial.println("Valve 1 State: OFF");
    delay(1000);
    updateShiftRegister();
    request->redirect("/");
  });

  server.on("/27/on", HTTP_GET, [](AsyncWebServerRequest *request) {
    output27State = true;
    updateShiftRegister();
    delay(1000);
    digitalWrite(valveEnablePin, HIGH);
    Serial.println("Valve 2 State: ON");
    request->redirect("/");
  });

  server.on("/27/off", HTTP_GET, [](AsyncWebServerRequest *request) {
    output27State = false;
    digitalWrite(valveEnablePin, LOW);
    Serial.println("Valve 2 State: OFF");
    delay(1000);
    updateShiftRegister();
    request->redirect("/");
  });

  server.on("/28/on", HTTP_GET, [](AsyncWebServerRequest *request) {
    output28State = true;
    updateShiftRegister();
    delay(1000);
    digitalWrite(valveEnablePin, HIGH);
    Serial.println("Valve 3 State: ON");
    request->redirect("/");
  });

  server.on("/28/off", HTTP_GET, [](AsyncWebServerRequest *request) {
    output28State = false;
    digitalWrite(valveEnablePin, LOW);
    Serial.println("Valve 3 State: OFF");
    delay(1000);
    updateShiftRegister();
    request->redirect("/");
  });

  server.on("/29/on", HTTP_GET, [](AsyncWebServerRequest *request) {
    output29State = true;
    updateShiftRegister();
    delay(1000);
    digitalWrite(valveEnablePin, HIGH);
    Serial.println("Valve 4 State: ON");
    request->redirect("/");
  });

  server.on("/29/off", HTTP_GET, [](AsyncWebServerRequest *request) {
    output29State = false;
    digitalWrite(valveEnablePin, LOW);
    Serial.println("Valve 4 State: OFF");
    delay(1000);
    updateShiftRegister();
    request->redirect("/");
  });

  server.on("/30/on", HTTP_GET, [](AsyncWebServerRequest *request) {
    output30State = true;
    updateShiftRegister();
    delay(1000);
    digitalWrite(valveEnablePin, HIGH);
    Serial.println("Valve 5 State: ON");
    request->redirect("/");
  });

  server.on("/30/off", HTTP_GET, [](AsyncWebServerRequest *request) {
    output30State = false;
    digitalWrite(valveEnablePin, LOW);
    Serial.println("Valve 5 State: OFF");
     delay(1000);

       updateShiftRegister();
    request->redirect("/");
  });

  server.on("/31/on", HTTP_GET, [](AsyncWebServerRequest *request) {
    output31State = true;
    updateShiftRegister();
    delay(1000);
    digitalWrite(valveEnablePin, HIGH);
    Serial.println("Valve 6 State: ON");
    request->redirect("/");
  });

  server.on("/31/off", HTTP_GET, [](AsyncWebServerRequest *request) {
    output31State = false;
    digitalWrite(valveEnablePin, LOW);
    Serial.println("Valve 6 State: OFF");
    delay(1000);
    updateShiftRegister();
    request->redirect("/");
  });

  server.on("/32/on", HTTP_GET, [](AsyncWebServerRequest *request) {
    output32State = true;
    updateShiftRegister();
    delay(1000);
    digitalWrite(valveEnablePin, HIGH);
    Serial.println("Valve 7 State: ON");
    request->redirect("/");
  });

  server.on("/32/off", HTTP_GET, [](AsyncWebServerRequest *request) {
    output32State = false;
    digitalWrite(valveEnablePin, LOW);
    Serial.println("Valve 7 State: OFF");
    delay(1000);
    updateShiftRegister();
    request->redirect("/");
  });

  server.on("/33/on", HTTP_GET, [](AsyncWebServerRequest *request){
    output33State = true;
    updateShiftRegister();
    delay(1000);
    digitalWrite(valveEnablePin, HIGH);
    Serial.println("Valve 8 State: ON");
    request->redirect("/");
  });

  server.on("/33/off", HTTP_GET, [](AsyncWebServerRequest *request) {
    output33State = false;
    digitalWrite(valveEnablePin, LOW);
    Serial.println("Valve 8 State: OFF");
    delay(1000);
    updateShiftRegister();
    request->redirect("/");
  });

  server.on("/flowcell/on", HTTP_GET, [](AsyncWebServerRequest *request) {
    flowcellEnableState = true;
    digitalWrite(flowcellEnablePin, HIGH);
    Serial.println("Flowcell Enable 1 State: ON");
    request->redirect("/");
  });

  server.on("/flowcell/off", HTTP_GET, [](AsyncWebServerRequest *request) {
    flowcellEnableState = false;
    digitalWrite(flowcellEnablePin, LOW);
    Serial.println("Flowcell Enable 1 State: OFF");
    request->redirect("/");
  });

  server.begin();
  Lowdriver_init();

   // Initialize PID controller settings
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 150);  // Limit output to lowdriver voltage range (0 to 150V)

// Set initial pump conditions for stable flow
  Lowdriver_setvoltage(150);  // Set initial voltage to 150V
  Lowdriver_setfrequency(100); // Set initial frequency to 100Hz
  delay(5000); // Allow time for the flow to stabilize

 // Set initial frequency to 100Hz
  Lowdriver_setfrequency(100);


  // Send column headers for PLX-DAQ
  Serial.println("LABEL,Time,Flow Rate (ml/min),Voltage (V)");
}

void updateShiftRegister() {
  byte ledsState = 0b00000000;
  if (output26State) ledsState |= 0b00000001;
  if (output27State) ledsState |= 0b00000010;
  if (output28State) ledsState |= 0b00000100;
  if (output29State) ledsState |= 0b00001000;
  if (output30State) ledsState |= 0b00010000;
  if (output31State) ledsState |= 0b00100000;
  if (output32State) ledsState |= 0b01000000;
  if (output33State) ledsState |= 0b10000000;

  digitalWrite(outputEnablePin, HIGH); // Disable outputs before updating
  digitalWrite(slaveSelectPin, LOW); // << RCLK line goes low
  SPI.transfer(ledsState); // << SRCLK goes high-low 8 times to output 8 bits of data
  digitalWrite(slaveSelectPin, HIGH); // Data outputs change on this rising edge << RCLK line goes high to move data into output register
  digitalWrite(outputEnablePin, LOW); // Enable outputs after updating

  Serial.print("ledsState: ");
  Serial.println(ledsState, BIN); // Print the binary representation of ledsState
}

// Function to select control registers
void selectControlRegisters() {
  Wire.beginTransmission(I2C_LOWDRIVER_ADRESS);
  Wire.write(0xFF);
  Wire.write(0x00);
  Wire.endTransmission();  
}

// Function to select memory registers
void selectMemoryRegisters() {
  Wire.beginTransmission(I2C_LOWDRIVER_ADRESS);
  Wire.write(0xFF);
  Wire.write(0x01);
  Wire.endTransmission();  
}

void Lowdriver_init() {
    selectControlRegisters();
    Wire.beginTransmission(I2C_LOWDRIVER_ADRESS);
    Wire.write(0x01); // Control Register setup
    Wire.write(0x02); // Set Gain
    Wire.write(0x00); // Wake up from standby
    Wire.write(0x01); // Play waveform sequence #1
    Wire.endTransmission();

    selectMemoryRegisters();
    Wire.beginTransmission(I2C_LOWDRIVER_ADRESS);
    Wire.write(0x00); // Start at Register 0x00
    Wire.write(0x05); // Set header size
    Wire.write(0x80); // Mode 3 start address
    Wire.write(0x06); // Start address lower byte
    Wire.write(0x00); // Stop address upper byte
    Wire.write(0x09); // Stop address lower byte
    Wire.write(0x00); // Repeat once
    Wire.write(0x00); // Initial Amplitude 0V
    Wire.write(0x0C); // Initial Frequency 100Hz
    Wire.write(100);  // Duration in cycles
    Wire.write(0x00); // No envelope
    Wire.endTransmission();
    delay(10);

    selectControlRegisters();
    Wire.beginTransmission(I2C_LOWDRIVER_ADRESS);
    Wire.write(0x02); // Set Go bit to start playback
    Wire.write(0x01);
    Wire.endTransmission();

    Serial.println("Lowdriver initialized");
    bPumpState = false;
    nPumpVoltageByte = 255;
}

// Set pump voltage with debugging and reinitialization
void Lowdriver_setvoltage(uint8_t _voltage) {
    Serial.print("Setting voltage to: ");
    Serial.println(_voltage);

    // Scale voltage for low-driver register
    float temp = _voltage;
    temp *= 255;
    temp /= 150;
    nPumpVoltageByte = constrain(temp, 0, 255);

    // Stop waveform playback
    Wire.beginTransmission(I2C_LOWDRIVER_ADRESS);
    Wire.write(0x02);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    // Set voltage value
    selectMemoryRegisters();
    Wire.beginTransmission(I2C_LOWDRIVER_ADRESS);
    Wire.write(0x06);
    Wire.write((bPumpState ? nPumpVoltageByte : 0));
    Wire.endTransmission();
    delay(10);

    // Restart waveform playback
    selectControlRegisters();
    Wire.beginTransmission(I2C_LOWDRIVER_ADRESS);
    Wire.write(0x02);
    Wire.write(0x01);
    Wire.endTransmission();
    delay(50);

    Serial.print("Voltage set to ");
    Serial.print(_voltage);
    Serial.print("V (Byte: ");
    Serial.print(nPumpVoltageByte);
    Serial.println(")");
}

// Set pump frequency with debugging and reinitialization
void Lowdriver_setfrequency(uint16_t frequency) {
    float temp = frequency / 7.8125;
    uint8_t frequencyByte = constrain(temp, 1, 255);

    Serial.print("Setting frequency to: ");
    Serial.print(frequency);
    Serial.print(" Hz (Byte: ");
    Serial.print(frequencyByte);
    Serial.println(")");

    // Stop waveform playback
    Wire.beginTransmission(I2C_LOWDRIVER_ADRESS);
    Wire.write(0x02);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    // Set frequency value
    selectMemoryRegisters();
    Wire.beginTransmission(I2C_LOWDRIVER_ADRESS);
    Wire.write(0x07);
    Wire.write(frequencyByte);
    Wire.endTransmission();
    delay(10);

    // Restart waveform playback
    selectControlRegisters();
    Wire.beginTransmission(I2C_LOWDRIVER_ADRESS);
    Wire.write(0x02);
    Wire.write(0x01);
    Wire.endTransmission();
    delay(50);

    Serial.println("Frequency set and waveform restarted");
    currentFrequency = frequency;
}

// Function to calculate moving average and update flow
void updateMovingAverage(double newFlowReading) {
    // Subtract the oldest reading from total
    totalFlow -= flowReadings[readIndex];
    
    // Replace oldest reading with the new reading
    flowReadings[readIndex] = newFlowReading;
    
    // Add the new reading to the total
    totalFlow += newFlowReading;
    
    // Advance to the next position in the array
    readIndex = (readIndex + 1) % numReadings;
    
    // Calculate the average
    averageFlow = totalFlow / numReadings;
}
void loop() {
    int ret = SLF3X.readSample();
    if (ret == 0) {
       // Get new flow reading and update moving average
        double newFlowReading = SLF3X.getFlow() * 0.0519 - 0.0122;//determined from sensor calibration, may need to be adapted if a different sensor is used
        updateMovingAverage(newFlowReading);  // Update moving average
        currentFlow = averageFlow;            // Use moving average for PID

        if (bPumpState) {
            Serial.println("Using Setpoint for PID: " + String(flowSetpoint));
            myPID.Compute();
            int roundedVoltage = round(pidOutput);  // Compute voltage from PID

             // Check if enough time has passed to update the voltage
            if (millis() - lastVoltageUpdateTime >= voltageUpdateInterval) {
                Lowdriver_setvoltage(roundedVoltage);  // Set pump voltage
                lastVoltageUpdateTime = millis();  // Update last time
            }
            // Set frequency to a constant value if static (e.g., 100 Hz)
            Lowdriver_setfrequency(100);

            // Display flow and voltage data
            Serial.print("Flow: ");
            Serial.print(currentFlow);
            Serial.print(" ml/min, Setpoint: ");
            Serial.print(flowSetpoint);
            Serial.print(" ml/min, Voltage: ");
            Serial.print(roundedVoltage);
            Serial.println("V");

            // Log data
            Serial.print("DATA,TIME,");
            Serial.print(currentFlow, 2);
            Serial.print(",");
            Serial.println(pidOutput, 2);
        } else {
            Lowdriver_setvoltage(0);
        }

        // Check for sensor alerts
        if (SLF3X.isAirInLineDetected()) Serial.println(" [Air in Line Detected]");
        if (SLF3X.isHighFlowDetected()) Serial.println(" [High Flow Detected]");
    } else {
        Serial.print("Error in SLF3X.readSample(): ");
        Serial.println(ret);
    }
    delay(MEASURE_DELAY);
}
