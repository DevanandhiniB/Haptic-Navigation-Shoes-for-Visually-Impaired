#include <SoftwareSerial.h>

SoftwareSerial SIM900A(10, 11);  // GSM module on pins 10 (RX), 11 (TX)
const int buttonPin = 6;         // Push button connected to pin 6

bool buttonPressed = false;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);  // Enable internal pull-up resistor
  SIM900A.begin(9600);   // GSM Module baud rate
  Serial.begin(9600);    // Serial Monitor baud rate

  Serial.println("Initializing SIM900A...");
  delay(2000);

  // Check if GSM module is responding
  SIM900A.println("AT");
  delay(1000);
  if (SIM900A.available()) {
    Serial.println("SIM900A Ready!");
  } else {
    Serial.println("Error: GSM Module not responding!");
  }

  Serial.println("Press button to send Emergency SMS and Call.");
}

void loop() {
  if (digitalRead(buttonPin) == LOW && !buttonPressed) {  // Button Pressed
    buttonPressed = true;
    Serial.println("Button Pressed! Sending SMS and Calling...");

    SendMessage();  // Send SMS
    delay(3000);    // Short delay before calling
    MakeCall();     // Make Call

    delay(5000);    // Prevent multiple triggers (debounce)
  }

  if (digitalRead(buttonPin) == HIGH) {
    buttonPressed = false;  // Reset button state when released
  }

  // Read GSM responses
  if (SIM900A.available()) {
    Serial.write(SIM900A.read());
  }
}

void SendMessage() {
  Serial.println("Sending Emergency SMS...");
  SIM900A.println("AT+CMGF=1");  
  delay(1000);
  SIM900A.println("AT+CMGS=\"+919037036273\"\r"); // Replace with actual number
  delay(1000);
  SIM900A.println("EMERGENCY!!!");  // Message content
  delay(100);
  SIM900A.println((char)26); // CTRL+Z to send
  delay(1000);
  Serial.println("SMS Sent Successfully!");
}

void MakeCall() {
  Serial.println("Dialing Emergency Number...");
  SIM900A.println("+919037036273;");  // Replace with actual number
  delay(15000); // Call duration (15 sec)
  SIM900A.println("ATH");  // Hang up
  Serial.println("Call Ended.");
}
