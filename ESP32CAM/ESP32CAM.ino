#include <WebServer.h>
#include <WiFi.h>
#include <esp32cam.h>
#include <CmdCode.h>
#include <ESP32Communication.h>
#include <memory>

#define SSID "NGUYENBAO"
// WiFi password hidden for security
#define PASSWORD "your_password_here"

#define PORT 80
#define CAMERA_MODEL_AI_THINKER
#define RS485_WRITE_EN_PIN 2

WebServer server(PORT);
char RX_buffer[14];
int RX_index = 0;
char TX_buffer[14];
static auto MyRes = esp32cam::Resolution::find(240, 240);
std::unique_ptr<esp32cam::Frame> shared_frame;
SemaphoreHandle_t camMutex;
TaskHandle_t camTaskHandle;

// Return JPEG frame to client
void serverJpg()
{
    if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(100)))
    {
        if (shared_frame)
        {
            server.setContentLength(shared_frame->size());
            server.send(200, "image/jpeg");
            WiFiClient client = server.client();
            shared_frame->writeTo(client);
        }
        else
        {
            server.send(503, "", "");
        }
        xSemaphoreGive(camMutex);
    }
    else
    {
        server.send(503, "", "");
    }
}

// Handle HTTP command /robot?cmd=...&value1=...
void handleCmd()
{
    int cmd = server.arg("cmd").toInt();
    int value1 = server.arg("value1").toInt();
    int value2 = server.arg("value2").toInt();
    int value3 = server.arg("value3").toInt();
    int value4 = server.arg("value4").toInt();
    int value5 = server.arg("value5").toInt();
    int value6 = server.arg("value6").toInt();

    // Send the camera frame first
    serverJpg();

    // Send RS485 frame to STM32
    digitalWrite(RS485_WRITE_EN_PIN, HIGH);
    TX_buffer[0] = _START_;
    TX_buffer[1] = cmd;
    TX_buffer[2] = (value1 >> 8) & 0xFF;
    TX_buffer[3] = value1 & 0xFF;
    TX_buffer[4] = (value2 >> 8) & 0xFF;
    TX_buffer[5] = value2 & 0xFF;
    TX_buffer[6] = (value3 >> 8) & 0xFF;
    TX_buffer[7] = value3 & 0xFF;
    TX_buffer[8] = (value4 >> 8) & 0xFF;
    TX_buffer[9] = value4 & 0xFF;
    TX_buffer[10] = (value5 >> 8) & 0xFF;
    TX_buffer[11] = value5 & 0xFF;
    TX_buffer[12] = (value6 >> 8) & 0xFF;
    TX_buffer[13] = value6 & 0xFF;

    Serial.write(TX_buffer, sizeof(TX_buffer));
    digitalWrite(RS485_WRITE_EN_PIN, LOW);
}

// Turn flash LED on
void handleOn()
{
    digitalWrite(4, HIGH);
    server.send(200, "text/plain", "Flash On");
}

// Turn flash LED off
void handleOff()
{
    digitalWrite(4, LOW);
    server.send(200, "text/plain", "Flash Off");
}

// Camera task running on core 0
void camTask(void* pvParam)
{
    while (true)
    {
        auto frame = esp32cam::capture();
        if (frame)
        {
            if (xSemaphoreTake(camMutex, portMAX_DELAY))
            {
                shared_frame = std::move(frame);
                xSemaphoreGive(camMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Approx. 20 FPS
    }
}

void setup()
{
    int cnt = 0;

    pinMode(RS485_WRITE_EN_PIN, OUTPUT);
    Serial.begin(115200);

    using namespace esp32cam;
    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(MyRes);
    cfg.setBufferCount(2);
    cfg.setJpeg(80);

    bool ok = Camera.begin(cfg);
    delay(2000);

    pinMode(4, OUTPUT);

    // Connect WiFi
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        if (++cnt == 20)
        {
            WiFi.begin(SSID, PASSWORD);
        }
    }

    IPAddress ip = WiFi.localIP();

    // Send IP address to STM32 via RS485 until it responds READY
    do
    {
        digitalWrite(RS485_WRITE_EN_PIN, HIGH);
        TX_buffer[0] = _SEND_IP_;
        TX_buffer[1] = 0x00;
        TX_buffer[2] = ip[0];
        TX_buffer[3] = ip[1];
        TX_buffer[4] = ip[2];
        TX_buffer[5] = ip[3];
        for (int i = 6; i < 14; i++) TX_buffer[i] = 0x00;

        Serial.write(TX_buffer, sizeof(TX_buffer));
        digitalWrite(RS485_WRITE_EN_PIN, LOW);
        delay(100);

    } while (Serial.read() != _STM32_READY_);

    camMutex = xSemaphoreCreateMutex();

    // Start camera capture task on core 0
    xTaskCreatePinnedToCore(camTask, "CameraTask", 4096, NULL, 1, &camTaskHandle, 0);

    // Initialize web routes on core 1
    server.on("/robot", handleCmd);
    server.on("/on", handleOn);
    server.on("/off", handleOff);
    server.begin();
}

void loop()
{
    server.handleClient();
}

// Read UART RX without blocking
void serialEvent()
{
    while (Serial.available())
    {
        RX_buffer[RX_index] = Serial.read();
        if (RX_index > 10)
        {
            RX_index = 0;
            return;
        }
        RX_index++;
    }
}
