#ifndef LOGGER_H
#define LOGGER_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <iostream>
#endif

#ifdef ESP32_WIFI
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <WiFi.h>
#endif

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace roboost::logging
{
    class Logger
    {
    public:
        template <typename LoggerType, typename... Args>
        static Logger& get_instance(Args&&... args)
        {
            if (!instance_)
            {
                instance_ = std::make_unique<LoggerType>(std::forward<Args>(args)...);
            }
            return *instance_;
        }

        static Logger& get_instance()
        {
            if (!instance_)
            {
                //? Should a default logger be created here?
                throw std::runtime_error("Logger instance has not been initialized.");
            }
            return *instance_;
        }

        void info(const std::string& message) { log("[INFO] " + message); }
        void warn(const std::string& message) { log("[WARN] " + message); }
        void error(const std::string& message) { log("[ERROR] " + message); }
        void debug(const std::string& message) { log("[DEBUG] " + message); }

    protected:
        virtual void log(const std::string& message) = 0; // This results in call overhead, but allows for easy extension. Should not be a problem for most applications.

    private:
        static std::unique_ptr<Logger> instance_;
    };

    std::unique_ptr<Logger> Logger::instance_ = nullptr;

#ifdef ARDUINO
#ifdef TEENSYDUINO
    using SerialType = usb_serial_class;
#else
    using SerialType = HardwareSerial;
#endif
    class SerialLogger : public Logger
    {
    public:
        SerialLogger(SerialType& Serial) : serial(&Serial) {}

        void set_serial(SerialType& serial) { this->serial = &serial; }

    protected:
        void log(const std::string& message) override
        {
            if (serial)
            {
                serial->println(message.c_str());
            }
        }

    private:
        SerialType* serial;
    };

#ifdef ESP32_WIFI
    class WebLogger : public Logger
    {
    public:
        WebLogger(const char* ssid, const char* password)
        {
            server = std::make_unique<AsyncWebServer>(80);
            events = std::make_unique<AsyncEventSource>("/events");
            setup(ssid, password);
        }

    protected:
        void log(const std::string& message) override { events->send(message.c_str(), "new_log", millis()); }

    private:
        std::unique_ptr<AsyncWebServer> server;
        std::unique_ptr<AsyncEventSource> events;

        void setup(const char* ssid, const char* password)
        {
            setup_SPIFFS();
            setup_wiFi(ssid, password);
            setup_server();
        }

        void setup_wiFi(const char* ssid, const char* password)
        {
            WiFi.begin(ssid, password);
            while (WiFi.status() != WL_CONNECTED)
            {
                delay(1000);
                Serial.print("Connecting to WiFi...");
            }
            Serial.println("Connected to WiFi");
            Serial.print("IP Address: ");
            Serial.println(WiFi.localIP());
        }

        void setup_server()
        {
            server->on("/", HTTP_GET,
                       [&](AsyncWebServerRequest* request)
                       {
                           if (SPIFFS.exists("/index.html"))
                           {
                               request->send(SPIFFS, "/index.html", "text/html");
                           }
                           else
                           {
                               request->send(404, "text/plain", "File not found");
                           }
                       });

            events->onConnect([&](AsyncEventSourceClient* client) { client->send("connected", NULL, millis()); });

            server->addHandler(events.get());
            server->begin();
        }

        void setup_SPIFFS()
        {
            if (!SPIFFS.begin(true))
            {
                if (!SPIFFS.format())
                {
                    return;
                }
                if (!SPIFFS.begin())
                {
                    return;
                }
            }

            if (SPIFFS.exists("/index.html"))
            {
                File file = SPIFFS.open("/index.html", "r");
                if (file)
                {
                    while (file.available())
                    {
                        Serial.write(file.read());
                    }
                    file.close();
                }
            }
        }
    };
#endif // ESP32_WIFI

#else
    class ConsoleLogger : public Logger
    {
    protected:
        void log(const std::string& message) override { std::cout << message << std::endl; }
    };
#endif // ARDUINO

} // namespace roboost::logging
#endif // LOGGER_H
