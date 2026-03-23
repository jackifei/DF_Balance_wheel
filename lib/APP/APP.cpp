/*
  ____        _                         _____
 |  _ \      | |                       / ____|
 | |_) | __ _| | __ _ _ __   ___ ___  | |     __ _ _ __
 |  _ < / _` | |/ _` | '_ \ / __/ _ \ | |    / _` | '__|
 | |_) | (_| | | (_| | | | | (_|  __/ | |___| (_| | |
 |____/ \__,_|_|\__,_|_| |_|\___\___|  \_____\__,_|_|
   _____           _           _   ____         __     __  _    _                 _     _
  / ____|         | |         | | |  _ \        \ \   / / | |  | |               | |   | |
 | |     ___  __ _| |_ ___  __| | | |_) |_   _   \ \_/ ___| |__| | __ _ _ __ ___ | | __| |
 | |    / _ \/ _` | __/ _ \/ _` | |  _ <| | | |   \   / _ |  __  |/ _` | '__/ _ \| |/ _` |
 | |___|  __| (_| | ||  __| (_| | | |_) | |_| |    | |  __| |  | | (_| | | | (_) | | (_| |
  \_____\___|\__,_|\__\___|\__,_| |____/ \__, |    |_|\___|_|  |_|\__,_|_|  \___/|_|\__,_|
                                          __/ |
                                         |___/
  Copyright (c) 2024 YeHarold
*/

#include "APP.h"
#include "ButtonAndBattery.h"
#include "BuzzerSound.h"
#include "RGBLED.h"
#include "UserConfig.h"

const char *ssid = USER_SSID;
const char *password = USER_PASSWORD;

AppControl_t appCTRL;

WebServer appServer(80);

#if STATIC_IP_MODE /*静态IP配置*/

// 使用这些宏来初始化IPAddress实例
IPAddress staticIP(STATIC_IP_FIRST_OCTET, STATIC_IP_SECOND_OCTET, STATIC_IP_THIRD_OCTET, STATIC_IP_FOURTH_OCTET);
IPAddress gateway(GATEWAY_FIRST_OCTET, GATEWAY_SECOND_OCTET, GATEWAY_THIRD_OCTET, GATEWAY_FOURTH_OCTET);
IPAddress subnet(SUBNET_FIRST_OCTET, SUBNET_SECOND_OCTET, SUBNET_THIRD_OCTET, SUBNET_FOURTH_OCTET);
#endif

static void AppServerTask(void *pvParameters)
{
    Serial.begin(115200);

#if STATIC_IP_MODE /*静态IP配置*/
    if (WiFi.config(staticIP, gateway, subnet) == false)
    {
        Serial.println(" Static IP Config Failed.");
    }
#endif

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    // #if !STATIC_IP_MODE
    Serial.println(" ");
    Serial.println("********************************************************");
    Serial.println(WiFi.localIP());
    Serial.println("********************************************************");
    // #endif

    appServer.on("/connect", appConnectHandler);
    appServer.on("/voltage", appVoltageHandler);
    appServer.on("/mpuset", appMPUsetHandler);
    appServer.on("/move", appMoveHandler);
    appServer.on("/poweroff", appPowerOffHandler);
    appServer.on("/rgb", appRGBChangeHandler);
    appServer.begin();
    Serial.println("Balance Car AppServer Started");

    for (;;)
    {
        appServer.handleClient();
        vTaskDelay(10);
    }
};

static void CarBrakeTask(void *pvParameters)
{
    for (;;)
    {
        if (appCTRL.Direction == "stop")
        {
            if (appCTRL.Velocity > appCTRL.MPUOffset)
            {
                appCTRL.Velocity -= BRAKE_DECAY;
                appCTRL.Velocity = (appCTRL.Velocity < appCTRL.MPUOffset) ? appCTRL.MPUOffset : appCTRL.Velocity;
            }
            if (appCTRL.Velocity < appCTRL.MPUOffset)
            {
                appCTRL.Velocity += BRAKE_DECAY;
                appCTRL.Velocity = (appCTRL.Velocity > appCTRL.MPUOffset) ? appCTRL.MPUOffset : appCTRL.Velocity;
            }
            // -----------------
            appCTRL.SteerVelocity = 0.0f;
        }
        else if (appCTRL.Direction == "mpu")
        {
            appCTRL.Velocity = appCTRL.MPUOffset;
        }
        vTaskDelay(10);
    }
};

void AppTaskInit::startTask()
{
    xTaskCreatePinnedToCore(AppServerTask, "App Sever Task", 6144, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(CarBrakeTask, "Brake Car Task", 2048, NULL, 1, NULL, 0);
};

/*-------------------------------------------------------------------------*/

static void appConnectHandler()
{
    appServer.send(200, "text/plain", "1");
};

static void appVoltageHandler()
{
    appServer.send(200, "text/plain", String(batteryVoltage));
};

static void appMPUsetHandler()
{
    appCTRL.Direction = appServer.arg("direction");
    appCTRL.MPUOffset = appServer.arg("distance").toFloat();
};

static void appMoveHandler()
{
    appCTRL.Direction = appServer.arg("direction");
    if (appCTRL.Direction == "up")
    {
        appCTRL.Velocity += MOVE_VEL;
    }

    if (appCTRL.Direction == "down")
    {
        appCTRL.Velocity -= MOVE_VEL;
    }

    if (appCTRL.Direction == "left")
    {
        appCTRL.SteerVelocity = (appCTRL.SteerVelocity > STR_LIMIT) ? STR_LIMIT : appCTRL.SteerVelocity + STR_VEL;
        appCTRL.Velocity = (appCTRL.Velocity > STR_LIMIT) ? STR_LIMIT : appCTRL.Velocity + STR_VEL;
    }

    if (appCTRL.Direction == "right")
    {
        appCTRL.SteerVelocity = (appCTRL.SteerVelocity < -STR_LIMIT) ? -STR_LIMIT : appCTRL.SteerVelocity - STR_VEL;
        appCTRL.Velocity = (appCTRL.Velocity < -STR_LIMIT) ? -STR_LIMIT : appCTRL.Velocity - STR_VEL;
    }
};

static void appPowerOffHandler()
{
    balanceCarPowerOff();
}

static void appRGBChangeHandler()
{

    appCTRL.Color = appServer.arg("color");
    appCTRL.RGBStatus = appServer.arg("status");

    if (appCTRL.RGBStatus == "on")
    {
        buzzer.play(changeColorID);
        rgb.ResetColor(appCTRL.Color);
    }
    else if (appCTRL.RGBStatus == "off")
    {
        turnOffRGB();
    }
}

AppTaskInit APP;