#include <Wire.h>

#define I2C_LCD_ADDRESS 0x3C  // I2C address of the SSD1306 display
#define I2C_LCD_SDA 12        // Use GPIO 12 as SDA
#define I2C_LCD_SCL 9         // Use GPIO 9 as SCL

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Display buffer to hold pixel data (128 columns x 64 rows / 8 bits per page)
uint8_t displayBuffer[SCREEN_WIDTH * SCREEN_HEIGHT / 8];

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

// Function prototypes
void ssd1306_init();
void ssd1306_clearBuffer();
void ssd1306_updateDisplay();
void ssd1306_togglePixel(uint8_t x, uint8_t y);
void ssd1306_drawLine(int x1, int y1, int x2, int y2);
void drawBlock(const int block[][4], size_t size);
void DisplayBattery(uint8_t percentage);
void DrawBatteryFrame();

void setup() {
  Serial.begin(115200);  // Initialize serial communication for debugging

  // Set I2C pins
  Wire.setSDA(I2C_LCD_SDA);
  Wire.setSCL(I2C_LCD_SCL);
  Wire.begin();  // Initialize I2C communication

  // Initialize the SSD1306
  ssd1306_init();

  // Clear the display buffer
  ssd1306_clearBuffer();

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

void loop() {
  for (int i = 0; i <= 100; i++) {
    Serial.println(i);
    ssd1306_clearBuffer();  // Clear the display buffer

    // Draw the battery frame
    DrawBatteryFrame();

    // Display the battery based on the percentage
    DisplayBattery(i);

    // Update the display with the new buffer
    ssd1306_updateDisplay();

    delay(50);  // Adjust delay for visible animation

    // Reverse direction from 100 back to 0
    if (i == 100) {
      for (int j = 100; j >= 0; j--) {
        Serial.println(j);
        ssd1306_clearBuffer();  // Clear the display buffer

        // Draw the battery frame
        DrawBatteryFrame();

        // Display the battery based on the percentage
        DisplayBattery(j);

        // Update the display with the new buffer
        ssd1306_updateDisplay();

        delay(50);  // Adjust delay for visible animation
      }
    }
  }
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
  if (percentage >= 75) {
    drawBlock(block4, sizeof(block4) / sizeof(block4[0]));
    drawBlock(block3, sizeof(block3) / sizeof(block3[0]));
    drawBlock(block2, sizeof(block2) / sizeof(block2[0]));
    drawBlock(block1, sizeof(block1) / sizeof(block1[0]));
  } else if (percentage >= 50) {
    drawBlock(block3, sizeof(block3) / sizeof(block3[0]));
    drawBlock(block2, sizeof(block2) / sizeof(block2[0]));
    drawBlock(block1, sizeof(block1) / sizeof(block1[0]));
  } else if (percentage >= 25) {
    drawBlock(block2, sizeof(block2) / sizeof(block2[0]));
    drawBlock(block1, sizeof(block1) / sizeof(block1[0]));
  } else if (percentage > 0) {
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