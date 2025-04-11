#define TINY_GSM_MODEM_SIM7600
#define DEBUG false
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_MOSI 9
#define OLED_CLK 10
#define OLED_DC 11
#define OLED_CS 12
#define OLED_RESET 13

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// Function to display a message on the OLED
#define GPS_CHECK_INTERVAL 1000 // Check GPS status every 1000 ms

#include <PubSubClient.h>
#include <TinyGsmClient.h>

// LTE Modem pins
#define LTE_PWRKEY_PIN 5
#define LTE_RESET_PIN 6
#define LTE_FLIGHT_PIN 7

// Switch pins
const int switch1Pin = 2;
const int switch2Pin = 3;

// Flags for button presses
volatile bool sosPressed = false;
volatile bool okPressed = false;
volatile bool sendingMessage = false;

// APN and MQTT credentials
const char apn[] = "airtelgprs.com";
const char broker[] = "34.100.196.132";
const int brokerPort = 1883;

const char topicTracking[] = "sim7600/nmea";
const char topicSOS[] = "sim7600/sos";
const char topicOK[] = "sim7600/ok";

// Intervals and timeouts
const unsigned long TRACK_SEND_INTERVAL = 1500;    // 1.5 seconds
const unsigned long MQTT_RETRY_INTERVAL = 8000;    // 8 seconds
const unsigned long GPRS_RETRY_INTERVAL = 10000;   // 10 seconds
const int MAX_MQTT_ATTEMPTS = 5;
const int MAX_GPRS_ATTEMPTS = 3;

// Timing variables
unsigned long lastTrackSendTime = 0;
unsigned long lastMqttRetryTime = 0;
unsigned long lastGprsRetryTime = 0;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
const unsigned long debounceDelay = 50;

// Connection state tracking
bool isConnecting = false;

// GSM and MQTT clients
TinyGsm modem(Serial1);
TinyGsmClient gsmClient(modem);
PubSubClient mqttClient(gsmClient);

bool gpsConnected = false; // Simulated GPS connection status
unsigned long previousMillis = 0;
bool blinkState = false;

void displayMessage(const String &message) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(message);
    display.display();
}

String extractNMEA(String response) {
    int start = response.indexOf("+CGPSINFO: ");
    if (start != -1) {
        start += 11;
        int end = response.indexOf("OK", start);
        if (end != -1 && end > start) {
            String nmea = response.substring(start, end);
            nmea.trim();
            return nmea;
        }
    }
    return "";
}

String sendData(String command, const int timeout, boolean debug = false) {
    String response = "";
    Serial1.println(command);

    long int startTime = millis();
    while (millis() - startTime < timeout) {
        while (Serial1.available()) {
            char c = Serial1.read();
            response += c;
        }
    }

    if (debug) {
        SerialUSB.print(command);
        SerialUSB.print(" Response: ");
        SerialUSB.println(response);
    }

    return response;
}
bool setupGPRS() {
    int attempts = 0;
    unsigned long retryInterval = GPRS_RETRY_INTERVAL;
    
    while (attempts < MAX_GPRS_ATTEMPTS) {
        SerialUSB.print("GPRS connection attempt ");
        SerialUSB.println(attempts + 1);
        
        if (modem.isGprsConnected()) {
            SerialUSB.println("GPRS already connected");
            return true;
        }

        if (modem.gprsConnect(apn, "", "")) {
            SerialUSB.println("GPRS connected successfully");
            return true;
        }

        attempts++;
        SerialUSB.println("GPRS connection failed, retrying...");
        delay(retryInterval);

        retryInterval += 2000; // Increment retry interval dynamically
    }
    
    SerialUSB.println("GPRS connection failed after maximum attempts");
    return false;
}


bool connectMQTT() {
    if (mqttClient.connected()) {
        return true;
    }

    if (!modem.isGprsConnected()) {
        SerialUSB.println("GPRS not connected, attempting to reconnect...");
        if (!setupGPRS()) {
            return false;
        }
    }

    String clientId = "TrackingClient-";
    clientId += String(random(0xffff), HEX);

    SerialUSB.print("Attempting MQTT connection with client ID: ");
    SerialUSB.println(clientId);

    if (mqttClient.connect(clientId.c_str())) {
        SerialUSB.println("Connected to MQTT broker");
        return true;
    }

    SerialUSB.print("MQTT connection failed, rc=");
    SerialUSB.println(mqttClient.state());
    return false;
}

void maintainConnection() {
    static int mqttAttempts = 0;
    unsigned long currentTime = millis();

    if (!mqttClient.connected() && !isConnecting) {
        if (currentTime - lastMqttRetryTime >= MQTT_RETRY_INTERVAL) {
            isConnecting = true;
            
            SerialUSB.print("Connection attempt ");
            SerialUSB.println(mqttAttempts + 1);

            if (connectMQTT()) {
                mqttAttempts = 0;
                isConnecting = false;
            } else {
                mqttAttempts++;
                if (mqttAttempts >= MAX_MQTT_ATTEMPTS) {
                    SerialUSB.println("Maximum MQTT connection attempts reached");
                    SerialUSB.println("Resetting modem...");
                    mqttAttempts = 0;
                    initModem();
                }
            }

            lastMqttRetryTime = currentTime;
            isConnecting = false;
        }
    } else if (mqttClient.connected()) {
        mqttAttempts = 0;
    }
}

void publishMessage(const char* topic, const String& payload) {
    if (!mqttClient.connected()) {
        SerialUSB.println("MQTT not connected. Attempting to reconnect...");
        maintainConnection();
    }
    
    if (mqttClient.connected() && mqttClient.publish(topic, payload.c_str())) {
        SerialUSB.println(String("Message published to ") + topic + ": " + payload);
    } else {
        SerialUSB.println(String("Failed to publish message to ") + topic);
    }
}

void handleSosMessage() {
    sendingMessage = true;
    sosPressed = false;
    displayMessage("Sending SOS...");
    SerialUSB.println("Sending SOS message...");
    String payload = "{\"carId\":2, \"message\": \"SOS\"}";

    for (int attempt = 0; attempt < 25; attempt++) {
        if (mqttClient.connected() || connectMQTT()) {
            publishMessage(topicSOS, payload);
             displayMessage("SOS Sent");
            SerialUSB.println("SOS sent successfully.");
            break;
        }
        delay(200);
    }

    sendingMessage = false;
}

void handleOkMessage() {
    sendingMessage = true;
    okPressed = false;
    displayMessage("Sending OK...");
    SerialUSB.println("Sending OK message...");
    String payload = "{\"carId\":2, \"message\": \"OK\"}";

    for (int attempt = 0; attempt < 25; attempt++) {
        if (mqttClient.connected() || connectMQTT()) {
            publishMessage(topicOK, payload);
            displayMessage("OK Sent");
            SerialUSB.println("OK sent successfully.");
            break;
        }
        delay(200);
    }

    sendingMessage = false;
}

void sendTrackData() {
    if (sendingMessage) return;

    String gpsInfo = sendData("AT+CGPSINFO", 1300, DEBUG);
    String nmeaSentence = extractNMEA(gpsInfo);
    SerialUSB.println(nmeaSentence + " nmea");

    if (nmeaSentence.length() > 8) {
        SerialUSB.println("Publishing Track data to MQTT");
        String payload = "{\"nmea\": \"" + nmeaSentence + "\",\"carId\":2}";
        publishMessage(topicTracking, payload);
    }
}

void initModem() {
    SerialUSB.println("Initializing modem...");
    modem.restart();
    delay(3000);
    
    if (setupGPRS()) {
        SerialUSB.println("Modem initialized successfully");
    } else {
        SerialUSB.println("Modem initialization failed");
    }

    SerialUSB.println("Enabling GPS...");
    sendData("AT+CGPS=0", 3000, DEBUG);
    sendData("AT+CGPS=1", 3000, DEBUG);
}

void setup() {
    SerialUSB.begin(115200);
    Serial1.begin(115200);
    randomSeed(analogRead(0));
if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_CS)) {
        SerialUSB.println(F("OLED initialization failed"));
        while (true);
    }
    displayMessage("Initializing...");

    pinMode(LTE_RESET_PIN, OUTPUT);
    digitalWrite(LTE_RESET_PIN, LOW);

    pinMode(LTE_PWRKEY_PIN, OUTPUT);
    digitalWrite(LTE_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(LTE_PWRKEY_PIN, HIGH);
    delay(2000);
    digitalWrite(LTE_PWRKEY_PIN, LOW);

    pinMode(LTE_FLIGHT_PIN, OUTPUT);
    digitalWrite(LTE_FLIGHT_PIN, LOW);

    pinMode(switch1Pin, INPUT_PULLUP);
    pinMode(switch2Pin, INPUT_PULLUP);

    while (!SerialUSB) {
        delay(10);
    }

    initModem();
    mqttClient.setServer(broker, brokerPort);
     display.clearDisplay();
    display.display();

    // Initial display message
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("GPS Status:");
    display.display();
        displayMessage("Setup complete");
}

void loop() {
    maintainConnection();
    mqttClient.loop();

    if (digitalRead(switch1Pin) == LOW) {
        if (millis() - lastDebounceTime1 > debounceDelay) {
            lastDebounceTime1 = millis();
            sosPressed = true;
        }
    }

    if (digitalRead(switch2Pin) == LOW) {
        if (millis() - lastDebounceTime2 > debounceDelay) {
            lastDebounceTime2 = millis();
            okPressed = true;
        }
    }

    if (sosPressed) {
        handleSosMessage();
    }

    if (okPressed) {
        handleOkMessage();
    }

    unsigned long currentTime = millis();
    if (currentTime - lastTrackSendTime >= TRACK_SEND_INTERVAL) {
        lastTrackSendTime = currentTime;
        sendTrackData();
    }
     if (millis() % 5000 < 2500) {
        gpsConnected = true;
    } else {
        gpsConnected = false;
    }

    // Check GPS status and update the display periodically
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= GPS_CHECK_INTERVAL) {
        previousMillis = currentMillis;

        if (gpsConnected) {
            blinkState = !blinkState; // Toggle blink state
        } else {
            blinkState = false; // Static empty circle when disconnected
        }

        updateDisplay();
    }
}


void updateDisplay() {
    display.clearDisplay();

    // Display GPS connection status text
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("GPS Status:");

    // Display blinking circle for connected or empty circle for disconnected
    if (gpsConnected) {
        display.fillCircle(64, 32, 10, SSD1306_WHITE); // Filled circle
    } else {
        display.drawCircle(64, 32, 10, SSD1306_WHITE); // Empty circle
    }

    display.display();
}
