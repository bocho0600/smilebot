/* 
Made by Kelvin Le
*/

#include <Servo.h>
#include <Wire.h>

#define I2C_ADDRESS 0x08  // I2C address of the device
#define I2C_SDA 26        // Use GPIO 26 as SDA
#define I2C_SCL 27        // Use GPIO 27 as SCL
#define SW2 20
#define BUZZ 16
#define LED0 25  // GPIO 25 for onboard LED
#define LED1 17
#define LED2 18
#define LED3 19
#define SEV1 4  // Servo Motors Pins
#define SEV2 5
#define SEV3 6
#define SEV4 7
#define EN1 0
#define PHS1 1
#define EN2 2
#define PHS2 3
#define I2C_LCD_ADDRESS 0x3C  // I2C address of the SSD1306 display
#define I2C_LCD_SDA 12        // Use GPIO 12 as SDA
#define I2C_LCD_SCL 9         // Use GPIO 9 as SCL

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define BATTERY_INPUT 28

// Display buffer to hold pixel data (128 columns x 64 rows / 8 bits per page)
uint8_t displayBuffer[SCREEN_WIDTH * SCREEN_HEIGHT / 8];

const int buzzerPin = 16;  // Pin connected to the buzzer

// Define blocks of lines
const int block1[][4] = {
  { 125, 2, 125, 8 },
  { 124, 2, 124, 8 },
  { 123, 2, 123, 8 },
  { 122, 2, 122, 8 }
};

const int block2[][4] = {
  { 120, 2, 120, 8 },
  { 119, 2, 119, 8 },
  { 118, 2, 118, 8 },
  { 117, 2, 117, 8 }
};

const int block3[][4] = {
  { 115, 2, 115, 8 },
  { 114, 2, 114, 8 },
  { 113, 2, 113, 8 },
  { 112, 2, 112, 8 }
};

const int block4[][4] = {
  { 110, 2, 110, 8 },
  { 109, 2, 109, 8 },
  { 108, 2, 108, 8 },
  { 107, 2, 107, 8 }
};

// Define predefined lines
const int predefinedLines[][4] = {
  { 127, 0, 106, 0 },
  { 127, 1, 127, 9 },
  { 127, 10, 106, 10 },
  { 105, 0, 105, 10 },
  { 104, 3, 104, 7 },
  { 103, 3, 103, 7 }
};


Servo Servo1;  // create servo objects
Servo Servo2;
Servo Servo3;
Servo Servo4;

// Function prototypes
void ssd1306_init();
void ssd1306_clearBuffer();
void ssd1306_updateDisplay();
void ssd1306_togglePixel(uint8_t x, uint8_t y);
void ssd1306_drawLine(int x1, int y1, int x2, int y2);
void drawBlock(const int block[][4], size_t size);
void DisplayBattery(uint8_t percentage);
void DrawBatteryFrame();
void receiveEvent(int howMany);
void ControlSystem(uint8_t* command, int length);
void PlaySong();
uint8_t waitingflag = 1;

void setup() {
  // Attach servos
  Servo1.attach(SEV1);
  Servo2.attach(SEV2);
  Servo3.attach(SEV3);
  Servo4.attach(SEV4);

  // Set up I2C on specific pins for the Raspberry Pi Pico
  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  Wire1.begin(I2C_ADDRESS);  // Initialize I2C communication

  // Attach a function to the receive event
  Wire1.onReceive(receiveEvent);

  // Set I2C pins
  Wire.setSDA(I2C_LCD_SDA);
  Wire.setSCL(I2C_LCD_SCL);
  Wire.begin();  // Initialize I2C communication
    // Initialize the SSD1306
  ssd1306_init();

  // Clear the display buffer
  ssd1306_clearBuffer();

  Serial.begin(115200);  // Initialize serial communication for debugging

  // Setup LED 0,1,2,3 as output and turn LED off initially
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
  digitalWrite(LED0, HIGH);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(PHS1, OUTPUT);
  pinMode(PHS2, OUTPUT);
  pinMode(BATTERY_INPUT, INPUT);
  pinMode(buzzerPin, OUTPUT);

  // Draw predefined lines
  for (int i = 0; i < sizeof(predefinedLines) / sizeof(predefinedLines[0]); i++) {
    int x1 = predefinedLines[i][0];
    int y1 = predefinedLines[i][1];
    int x2 = predefinedLines[i][2];
    int y2 = predefinedLines[i][3];
    ssd1306_drawLine(x1, y1, x2, y2);
  }
  // Update the display initially
  ssd1306_updateDisplay();
}
int battery_value = 0;
int battery_status = 0;  //from 1 to 4
int prev_battery_status = 0;

void loop() {
  while (1) {
    int battery_value = analogRead(BATTERY_INPUT);  // Read the analog value (0–1023)
    if ((battery_value >= 600) && (battery_value < 680)) {
      battery_status = 1;
    } else if ((battery_value >= 680) && (battery_value < 750)) {
      battery_status = 2;
    } else if ((battery_value >= 750) && (battery_value < 830)) {
      battery_status = 3;
    } else if (battery_value >= 830) {
      battery_status = 4;
    }
    Serial.println(battery_value);
    if (battery_status != prev_battery_status) {  //Only update the LCD if the battery status is changed
      ssd1306_clearBuffer();  // Clear the display buffer
      DrawBatteryFrame(); // Draw the battery frame
      DisplayBattery(battery_status); // Display the battery based on the percentage
      ssd1306_updateDisplay(); // Update the display with the new buffer
      delay(50);
    }

    prev_battery_status = battery_status;  ///restore status
  }

}

void ConfigureLED(uint8_t led, int length, const char* text) {
  if ((text[4] == 'O') && (text[5] == 'N')) {
    digitalWrite(led, HIGH);  // Turn on LED
    Serial.println("LED turned ON");
  } else if ((text[4] == 'O') && (text[5] == 'F') && (text[6] == 'F')) {
    digitalWrite(led, LOW);  // Turn off LED
    Serial.println("LED turned OFF");
  } else {
    Serial.println("Invalid command for LED");
  }
}

void ControlSystem(uint8_t* command, int length) {
  // Convert the received command to a string
  char text[length + 1];
  for (int i = 0; i < length; i++) {
    text[i] = (char)command[i];  // Cast each integer to char and store in text
  }
  text[length] = '\0';  // Null-terminate the string
  Serial.println(text);

  switch (text[1]) {


    case 'M':
      {

        /* Example Command of Motor Control: 
              M1 1 255 -> Set speed of Motor 1 to full speed, Anticlockwise
              M2 0 0 -> Set speed of Motor 2 to low speed, Clockwise
              M1 S -> Stop motor 1
              #Syntax: M<ID> <Direction> <Speed>
              */

        if (length > 4) {            // Ensure the command has sufficient length
          char Direction = text[4];  // Get the direction ('0', '1', or 'S')

          if (Direction != 'S') {                                 // If not stop command
            char Speed[4] = { text[6], text[7], text[8], '\0' };  // Extract speed value
            int speed = atoi(Speed);                              // Convert speed to integer

            switch (text[2]) {
              case '1':
                if (Direction == '0') {
                  digitalWrite(PHS1, HIGH);  // Set Direction
                } else if (Direction == '1') {
                  digitalWrite(PHS1, LOW);  // Set Direction
                }
                analogWrite(EN1, speed);  // Write speed by generating PWM signal
                Serial.print("Motor 1 set to direction ");
                Serial.print(Direction);
                Serial.print(" with speed ");
                Serial.println(speed);
                break;

              case '2':
                if (Direction == '0') {
                  digitalWrite(PHS2, HIGH);  // Set Direction
                } else if (Direction == '1') {
                  digitalWrite(PHS2, LOW);  // Set Direction
                }
                analogWrite(EN2, speed);  // Write speed by generating PWM signal
                Serial.print("Motor 2 set to direction ");
                Serial.print(Direction);
                Serial.print(" with speed ");
                Serial.println(speed);
                break;

              default:
                Serial.println("Invalid motor index");
                break;
            }
          } else if (Direction == 'S') {  // Stop command 'S'
            switch (text[2]) {
              case '1':
                analogWrite(EN1, 0);  // Stop Motor 1
                Serial.println("Motor 1 stopped");
                break;
              case '2':
                analogWrite(EN2, 0);  // Stop Motor 2
                Serial.println("Motor 2 stopped");
                break;
              default:
                Serial.println("Invalid motor index");
                break;
            }
          }
        } else {
          Serial.println("Invalid command length for motor");
        }
        break;
      }

    case 'L':  // LEDs
      switch (text[2]) {
        case '0':
          ConfigureLED(LED0, length, text);
          break;
        case '1':
          ConfigureLED(LED1, length, text);
          break;
        case '2':
          ConfigureLED(LED2, length, text);
          break;
        case '3':
          ConfigureLED(LED3, length, text);
          break;
        default:
          Serial.println("Invalid LED index");
          break;
      }
      break;

    case 'S':                                                     // Servo Motors
      if (length > 3) {                                           // Ensure command length is sufficient
        char angleText[4] = { text[4], text[5], text[6], '\0' };  // Extract up to 3 digits and null-terminate
        int angle = atoi(angleText);                              // Convert to integer

        // Ensure angle is within the valid range
        angle = constrain(angle, 0, 360);

        switch (text[2]) {
          case '1':
            Servo1.write(angle);
            Serial.print("Adjusting Servo 1 to ");
            Serial.println(angle);
            break;
          case '2':
            Servo2.write(angle);
            Serial.print("Adjusting Servo 2 to ");
            Serial.println(angle);
            break;
          case '3':
            Servo3.write(angle);
            Serial.print("Adjusting Servo 3 to ");
            Serial.println(angle);
            break;
          case '4':
            Servo4.write(angle);
            Serial.print("Adjusting Servo 4 to ");
            Serial.println(angle);
            break;
          default:
            Serial.println("Invalid servo index");
            break;
        }
      } else {
        Serial.println("Invalid command length for servo");
      }
      break;

    default:
      Serial.println("Unknown command received");
      break;
  }
}

void receiveEvent(int howMany) {
  if (howMany >= 2) {
    waitingflag = 0;       // turn off waiting flag because command recieved
    uint8_t cmd[howMany];  // Buffer to store received command
    for (int i = 0; i < howMany; i++) {
      if (Wire1.available()) {
        cmd[i] = Wire1.read();  // Read received byte
      }
    }
    Serial.print("Received command: ");
    for (int i = 0; i < howMany; i++) {
      Serial.print((char)cmd[i]);
    }
    Serial.println();

    ControlSystem(cmd, howMany);
  }
}


int currentLED = 0;  // Keeps track of which LED to light up next

void turnOnNextLED() {
  // Turn off all LEDs first
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

  // Turn on the next LED in sequence
  if (currentLED == 0) {
    digitalWrite(LED1, HIGH);
  } else if (currentLED == 1) {
    digitalWrite(LED2, HIGH);
  } else if (currentLED == 2) {
    digitalWrite(LED3, HIGH);
  }

  // Update the currentLED variable to point to the next LED
  currentLED = (currentLED + 1) % 3;  // Cycle through 0, 1, 2
}


void ssd1306_init() {
  Wire.beginTransmission(I2C_LCD_ADDRESS);
  Wire.write(0x00);  // Command stream

  // Initialization sequence for 128x64 SSD1306
  Wire.write(0xAE);  // Display OFF (sleep mode)
  Wire.write(0xD5);  // Set display clock divide ratio/oscillator frequency
  Wire.write(0x80);  // The suggested ratio 0x80
  Wire.write(0xA8);  // Set multiplex ratio(1 to 64)
  Wire.write(0x3F);  // 64MUX (height - 1)
  Wire.write(0xD3);  // Set display offset
  Wire.write(0x00);  // No offset
  Wire.write(0x40);  // Set start line address to 0
  Wire.write(0x8D);  // Enable charge pump regulator
  Wire.write(0x14);
  Wire.write(0x20);  // Set Memory Addressing Mode
  Wire.write(0x00);  // Horizontal addressing mode
  Wire.write(0xA1);  // Set segment re-map 0 to 127
  Wire.write(0xC8);  // Set COM Output Scan Direction
  Wire.write(0xDA);  // Set COM Pins hardware configuration
  Wire.write(0x12);  // Alternative COM pin configuration for 128x64
  Wire.write(0x81);  // Set contrast control register
  Wire.write(0xCF);  // Contrast value
  Wire.write(0xD9);  // Set pre-charge period
  Wire.write(0xF1);
  Wire.write(0xDB);  // Set Vcomh deselect level
  Wire.write(0x40);
  Wire.write(0xA4);  // Entire display ON
  Wire.write(0xA6);  // Set normal display
  Wire.write(0xAF);  // Display ON in normal mode

  Wire.endTransmission();
}

void ssd1306_clearBuffer() {
  memset(displayBuffer, 0, sizeof(displayBuffer));
}

void ssd1306_updateDisplay() {
  // Set Column Address
  Wire.beginTransmission(I2C_LCD_ADDRESS);
  Wire.write(0x00);              // Command stream
  Wire.write(0x21);              // Set column address
  Wire.write(0);                 // Start at 0
  Wire.write(SCREEN_WIDTH - 1);  // End at 127

  // Set Page Address
  Wire.write(0x22);                     // Set page address
  Wire.write(0);                        // Start at 0
  Wire.write((SCREEN_HEIGHT / 8) - 1);  // End at 7 (for 64-pixel tall display)
  Wire.endTransmission();

  // Write the buffer to the display
  for (uint16_t i = 0; i < sizeof(displayBuffer); i += 16) {
    Wire.beginTransmission(I2C_LCD_ADDRESS);
    Wire.write(0x40);  // Data stream
    Wire.write(&displayBuffer[i], 16);
    Wire.endTransmission();
  }
}

void ssd1306_togglePixel(uint8_t x, uint8_t y) {
  if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) {
    // Out of bounds
    Serial.println("Coordinates out of bounds.");
    return;
  }

  uint16_t byteIndex = x + (y / 8) * SCREEN_WIDTH;
  uint8_t bitMask = 1 << (y % 8);

  // Toggle the pixel in the display buffer
  displayBuffer[byteIndex] ^= bitMask;
}

void ssd1306_drawLine(int x1, int y1, int x2, int y2) {
  // Bresenham's line algorithm
  int dx = abs(x2 - x1);
  int dy = -abs(y2 - y1);
  int sx = x1 < x2 ? 1 : -1;
  int sy = y1 < y2 ? 1 : -1;
  int err = dx + dy;

  while (true) {
    ssd1306_togglePixel(x1, y1);
    if (x1 == x2 && y1 == y2) break;
    int e2 = err * 2;
    if (e2 >= dy) {
      err += dy;
      x1 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y1 += sy;
    }
  }
}

void drawBlock(const int block[][4], size_t size) {
  for (int i = 0; i < size; i++) {
    int x1 = block[i][0];
    int y1 = block[i][1];
    int x2 = block[i][2];
    int y2 = block[i][3];
    ssd1306_drawLine(x1, y1, x2, y2);
  }
}

void DisplayBattery(uint8_t percentage) {
  if (percentage == 4) {
    drawBlock(block4, sizeof(block4) / sizeof(block4[0]));
    drawBlock(block3, sizeof(block3) / sizeof(block3[0]));
    drawBlock(block2, sizeof(block2) / sizeof(block2[0]));
    drawBlock(block1, sizeof(block1) / sizeof(block1[0]));
  } else if (percentage == 3) {
    drawBlock(block3, sizeof(block3) / sizeof(block3[0]));
    drawBlock(block2, sizeof(block2) / sizeof(block2[0]));
    drawBlock(block1, sizeof(block1) / sizeof(block1[0]));
  } else if (percentage == 2) {
    drawBlock(block2, sizeof(block2) / sizeof(block2[0]));
    drawBlock(block1, sizeof(block1) / sizeof(block1[0]));
  } else if (percentage == 1) {
    drawBlock(block1, sizeof(block1) / sizeof(block1[0]));
  }
}

void DrawBatteryFrame() {
  // Draw the battery frame
  for (int i = 0; i < sizeof(predefinedLines) / sizeof(predefinedLines[0]); i++) {
    int x1 = predefinedLines[i][0];
    int y1 = predefinedLines[i][1];
    int x2 = predefinedLines[i][2];
    int y2 = predefinedLines[i][3];
    ssd1306_drawLine(x1, y1, x2, y2);
  }
}
