#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const char* ssid = "Reno 8";
const char* password = "88888888";

const int irPin = 14;
int lastIRState = HIGH;
int objectCount = 0;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  pinMode(irPin, INPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED initialization failed!");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Connecting to WiFi...");
  display.display();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  String ipStr = WiFi.localIP().toString();
  Serial.println("\nWiFi connected");
  Serial.println("IP address: " + ipStr);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi Connected!");
  display.print("IP: ");
  display.println(ipStr);
  display.display();

  server.begin();
}

void loop() {
  int irValue = digitalRead(irPin);

  if (lastIRState == HIGH && irValue == LOW) {
    objectCount++;
    Serial.println("Object Detected! Count: " + String(objectCount));
  }
  lastIRState = irValue;

  display.setCursor(0, 40);
  display.fillRect(0, 40, 128, 24, BLACK); // Clear area
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.print("Object Count: ");
  display.println(objectCount);
  display.display();

  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    client.flush();

    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println();

    client.println("<!DOCTYPE html><html>");
    client.println("<head><meta charset='UTF-8'>");
    client.println("<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
    client.println("<meta http-equiv='refresh' content='3'>"); // Auto-refresh
    client.println("<title>ESP32 Object Counter</title>");
    client.println("<style>");
    client.println("body { font-family: Arial, sans-serif; background: #f0f8ff; text-align: center; padding: 40px; }");
    client.println(".card { background: white; padding: 20px; border-radius: 15px; box-shadow: 0 0 15px rgba(0,0,0,0.2); max-width: 400px; margin: auto; }");
    client.println("h1 { color: #0066cc; }");
    client.println("p { font-size: 1.2em; }");
    client.println("strong { font-size: 1.5em; color: #007700; }");
    client.println(".credits { margin-top: 20px; font-size: 0.95em; color: #444; }");
    client.println("</style>");
    client.println("</head><body>");
    client.println("<div class='card'>");
    client.println("<h1>🔢 Object Sorting & Countung</h1>");
    client.println("<p>📦 Objects Detected:</p>");
    client.print("<strong>");
    client.print(objectCount);
    client.println("</strong>");
    client.print("<p>🌐 IP Address: <br><strong>");
    client.print(WiFi.localIP());
    client.println("</strong></p>");
    client.println("<p>⏳ Page auto-refreshes every 3 seconds</p>");

    client.println("<div class='credits'>");
    client.println("<p><strong>FACULTY:</strong> Vidhyapathi CM</p>");
    client.println("<p><strong>Developed by:</strong><br>");
    client.println("Abhinaw Kumar Ojha (22BML0025)<br>");
    client.println("Kaniska U S (22BEC0289)<br>");
    client.println("R Keerthanaa (22BEC0291)<br>");
    client.println("Madhavakrishnan S (22BML0051)</p>");
    client.println("</div>");

    client.println("</div></body></html>");
    client.stop();
  }

  delay(100); 
}
