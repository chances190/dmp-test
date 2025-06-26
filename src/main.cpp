#include "mbed.h"

BufferedSerial serial_port(USBTX, USBRX, 115200);

int main() {
    serial_port.set_baud(115200);
    const char message[] = "Hello world\n";
    serial_port.write(message, sizeof(message));
    while (true) {
        ThisThread::sleep_for(1s);
    }
}

// #include <mbed.h>
// #include "ports_v5.0.h"
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"

// static BufferedSerial serial_port(USBTX, USBRX, 115200);
// FileHandle *mbed::mbed_override_console(int fd) { return &serial_port; }

// MPU6050 mpu;

// int main() {
//     serial_port.set_baud(115200);
//     printf("Initializing MPU6050...\r\n");
//     mpu.initialize();
//     if (!mpu.testConnection()) {
//         printf("MPU6050 connection failed\r\n");
//         while (true) { ThisThread::sleep_for(1s); }
//     }

//     printf("Initializing DMP...\r\n");
//     uint8_t devStatus = mpu.dmpInitialize();
//     if (devStatus != 0) {
//         printf("DMP Initialization failed (code %d)\r\n", devStatus);
//         while (true) { ThisThread::sleep_for(1s); }
//     }

//     mpu.setDMPEnabled(true);
//     printf("DMP ready\r\n");

//     uint16_t packetSize = mpu.dmpGetFIFOPacketSize();

//     uint8_t fifoBuffer[64];
//     Quaternion q;
//     VectorFloat gravity;
//     float ypr[3];

//     while (true) {
//         uint16_t fifoCount = mpu.getFIFOCount();
//         if (fifoCount >= packetSize) {
//             mpu.getFIFOBytes(fifoBuffer, packetSize);
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetGravity(&gravity, &q);
//             mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//             float yaw = ypr[0] * 180.0f / M_PI;
//             float pitch = ypr[1] * 180.0f / M_PI;
//             float roll = ypr[2] * 180.0f / M_PI;
//             printf("Y: %.2f\tP: %.2f\tR: %.2f\r\n", yaw, pitch, roll);
//         }
//         ThisThread::sleep_for(10ms);
//     }
// }