#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <TinyGPS++.h>

SoftwareSerial SIM7670Serial(2, 3);  // SIM7670 communication (RX, TX)
AltSoftSerial GPSSerial;             // GPS NEO-6M communication (RX=8, TX=9)
TinyGPSPlus gps;

int buzzerPin = 6;              // Pin for the buzzer
int buttonPin = 7;              // Pin for the push button
const int xPin = A0;            // ADXL335 X-axis pin
const int yPin = A1;            // ADXL335 Y-axis pin
const int zPin = A2;            // ADXL335 Z-axis pin
int threshold = 450;            // Acceleration threshold

int buttonPressCount = 0;       // To track button presses
bool preventSMS = false;        // Flag to prevent SMS after 3 button presses

void sendATCommand(const char* cmd, const char* expectedResponse, unsigned long timeout) {
  SIM7670Serial.println(cmd);
  String response = "";
  unsigned long startTime = millis();
  bool responseOK = false;

  while (millis() - startTime < timeout) {
    while (SIM7670Serial.available() > 0) {
      char c = SIM7670Serial.read();
      response += c;
    }
    if (response.indexOf(expectedResponse) != -1) {
      responseOK = true;
      break; // expected response found
    }
  }
  Serial.println(response);

  if (responseOK)
    Serial.println("Response OK");
  else
    Serial.println("Timeout without expected response");
}

void setup() {
  Serial.begin(115200);
  SIM7670Serial.begin(115200);   // Begin SIM7670 communication
  GPSSerial.begin(115200);         // Begin GPS communication
  
  pinMode(buzzerPin, OUTPUT);    // Buzzer output
  pinMode(buttonPin, INPUT_PULLUP); // Button input (with internal pull-up)
  
  sendATCommand("AT+CGMM", "OK", 1000);    // Check communication with the SIM7670
  sendATCommand("AT+CMGF=1", "OK", 1000);  // Set SMS format to text mode
}

void sendSMSWithLocation(String number, String message) {
  // Wait for a GPS fix
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }
  
  // If location data is available, append it to the message
  if (gps.location.isValid()) {
    message += "\nLocation: https://maps.google.com/?q=";
    message += String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  } else {
    message += "\nLocation: https://maps.google.com/?q=25.2543,87.0444";
  }

  String cmd = "AT+CMGS=\"" + number + "\"\r\n";
  SIM7670Serial.print(cmd);
  delay(100);  // Small delay before sending the message
  SIM7670Serial.print(message);  // Send the message text
  delay(100);  // Small delay after the message
  SIM7670Serial.write(0x1A);  // Send Ctrl+Z to indicate the end of the message
  delay(1000);  // Wait for SMS to be sent
}

bool detectAcceleration() {
  int x = analogRead(xPin);
  int y = analogRead(yPin);
  int z = analogRead(zPin);
  
  // Check if the acceleration exceeds the threshold
  return (abs(x - 512) > threshold || abs(y - 512) > threshold || abs(z - 512) > threshold);
}

void loop() {
  // Detect button presses and count them
  if (digitalRead(buttonPin) == LOW) {
    delay(100);  // Debounce
    if (digitalRead(buttonPin) == LOW) {
      buttonPressCount++;
      Serial.print("Button pressed: ");
      Serial.println(buttonPressCount);
      delay(500);  // Wait for button press to register
    }
  }

  // If button pressed 3 times, prevent sending SMS
  if (buttonPressCount >= 3) {
    preventSMS = true;
    Serial.println("Button pressed 3 times. SMS will not be sent.");
  }

  // Detect acceleration and trigger SMS if not prevented
  if (detectAcceleration() && !preventSMS) {
    Serial.println("Accident detected!");

    // Turn on the buzzer
    digitalWrite(buzzerPin, HIGH);
    unsigned long buzzerStart = millis();
    
    while (millis() - buzzerStart < 10000) {  // Wait for 3 seconds
      if (digitalRead(buttonPin) == LOW) {   // If the button is pressed during this time
        Serial.println("Button pressed, SMS not sent");
        digitalWrite(buzzerPin, LOW); // Turn off the buzzer
        return;  // Exit the loop, don't send the SMS
      }
    }

    // After 3 seconds, turn off the buzzer and send the SMS with location
    digitalWrite(buzzerPin, LOW);
    sendSMSWithLocation("+9180044-----", "Alert! Accident detected.");
    delay(3000);  // Wait for 3 seconds before detecting acceleration again
  }
}
