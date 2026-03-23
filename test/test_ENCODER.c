#include <Arduino.h>
#include <SimpleFOC.h>

// 分别测试编码器AB是否工作正常

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1); // 编码器A
// TwoWire I2Cone = TwoWire(0); // 编码器B

void setup()
{
    Serial.begin(115200);

    I2Ctwo.begin(37, 36, 400000UL); // 编码器A
    sensor.init(&I2Ctwo);           // 编码器A

    // I2Cone.begin(8, 9, 400000UL); // 编码器B
    // sensor.init(&I2Cone); // 编码器B

    Serial.println("Sensor ready");
    delay(1000);
}

void loop()
{
    sensor.update();
    Serial.print(sensor.getAngle());
    Serial.print("\t");
    Serial.println(sensor.getVelocity());
}
