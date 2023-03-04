//
// ARDUINO UNO Example Program
//
// Received the desired speed from the ros node via the serial line and
// stores it into the global variables. 
// A response is returend for connection alive check.

int16_t speed_left = 0;
int16_t speed_right = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // Read string until \n is received
  String rx_msg = Serial.readStringUntil('\n');

  // Export data from string
  sscanf(rx_msg.c_str(), "L%iR%i\n", &speed_left, &speed_right);

  // Respond to ros node
  Serial.print("Set: Speed-Left: ");
  Serial.print(speed_left);
  Serial.print(" [rpm] Speed Right: ");
  Serial.print(speed_right);
  Serial.println(" [rpm]");
}