
// void setup() {
//   BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
//   BP32.forgetBluetoothKeys();
//   motorSetup();
//   servoSetup();
// }

// IPAddress serverIP(192, 168, 50, 29);
// // const uint16_t serverPort = 10000;



void setup() {
  Serial.begin(115200);
  motorSetup();
  servoSetup();
  pinMode(led, OUTPUT);

  // Turn on the I2C power by pulling pin HIGH.
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
  Serial.println("Adafruit setuped");

  delay(500);
  Serial.println("DPS310");
  if (!dps.begin_I2C(0x77)) {  // Can pass in I2C address here
                               //if (! dps.begin_SPI(DPS310_CS)) {  // If you want to use SPI
    Serial.println("Failed to find DPS");
    while (1) yield();
  }

  delay(500);
  if (!bno08x.begin_I2C()) {
    //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
    //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) yield();
  }
  Serial.println("DPS and BNO08x ok ");

  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  setReports(reportType, reportIntervalUs);
  // onePixel.begin();            // Start the NeoPixel object
  // onePixel.clear();            // Set NeoPixel color to black (0,0,0)
  // onePixel.setBrightness(20);  // Affects all subsequent settings
  // onePixel.show();             // Update the pixel state
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());
  // Start the server
  server.begin();
}