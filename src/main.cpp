#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "ports_v5.0.h"
#include <mbed.h>

// Select the mode you want to test (uncomment only one)
// #define MODE_RAW_SENSORS    // Read raw accelerometer and gyroscope data
// #define MODE_FIFO           // Read data from the FIFO buffer
#define MODE_DMP            // Use the Digital Motion Processor for enhanced output

// Serial communication
static BufferedSerial serial(USBTX, USBRX, 115200);

// LED for activity indication
static DigitalOut led(PIN_LED_RED);
bool blinkState = false;

// MPU6050 instance
MPU6050 mpu;

// Timing control
Timer sampleTimer;
const int PRINT_INTERVAL_MS = 100;  // Print data every 100ms

// Buffer for DMP/FIFO data
uint8_t fifoBuffer[64];

// DMP variables (only used in DMP mode)
#ifdef MODE_DMP
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

    bool dmpReady = false;  // set true if DMP init was successful
    uint16_t packetSize;    // expected DMP packet size
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
#endif

// Function to calibrate the MPU6050 (works for all modes)
void calibrateMPU() {
    printf("Calibrating MPU6050, keep it still...\r\n");
    
    // Store offsets
    int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
    int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
    
    // Number of samples to take
    const int NUM_SAMPLES = 50;
    
    // Collect calibration data
    for (int i = 0; i < NUM_SAMPLES; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        // Accumulate readings
        ax_offset += ax;
        ay_offset += ay;
        az_offset += (az - 16384); // Remove 1g from Z axis (assuming horizontal orientation)
        gx_offset += gx;
        gy_offset += gy;
        gz_offset += gz;
        
        ThisThread::sleep_for(10ms);
    }
    
    // Calculate average offsets
    ax_offset /= NUM_SAMPLES;
    ay_offset /= NUM_SAMPLES;
    az_offset /= NUM_SAMPLES;
    gx_offset /= NUM_SAMPLES;
    gy_offset /= NUM_SAMPLES;
    gz_offset /= NUM_SAMPLES;
    
    // Set the offsets
    mpu.setXAccelOffset(-ax_offset);
    mpu.setYAccelOffset(-ay_offset);
    mpu.setZAccelOffset(-az_offset);
    mpu.setXGyroOffset(-gx_offset);
    mpu.setYGyroOffset(-gy_offset);
    mpu.setZGyroOffset(-gz_offset);
    
    printf("Calibration complete\r\n");
    printf("Offsets: accel X=%d Y=%d Z=%d, gyro X=%d Y=%d Z=%d\r\n", 
           -ax_offset, -ay_offset, -az_offset, -gx_offset, -gy_offset, -gz_offset);
}

// Function to initialize the MPU based on the selected mode
void setupMPU() {
    // Basic initialization for all modes
    printf("Initializing MPU6050...\r\n");
    mpu.initialize();
    
    // Verify connection
    if (mpu.testConnection()) {
        printf("MPU6050 connection successful\r\n");
    } else {
        printf("MPU6050 connection failed\r\n");
        while (true) {
            ThisThread::sleep_for(1s);
        }
    }
    
    // Configure sensor for each mode
    #ifdef MODE_RAW_SENSORS
        printf("Configuring for RAW SENSOR mode\r\n");
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);  // 2000 degrees/second
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // +/- 2g
        mpu.setDLPFMode(MPU6050_DLPF_BW_42);             // 42 Hz low-pass filter
        calibrateMPU();
    #endif
    
    #ifdef MODE_FIFO
        printf("Configuring for FIFO mode\r\n");
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        mpu.setDLPFMode(MPU6050_DLPF_BW_42);
        
        // Reset and configure FIFO
        mpu.resetFIFO();
        mpu.setFIFOEnabled(true);
        mpu.setAccelFIFOEnabled(true);
        mpu.setXGyroFIFOEnabled(true);
        mpu.setYGyroFIFOEnabled(true);
        mpu.setZGyroFIFOEnabled(true);
        
        calibrateMPU();
    #endif
    
    #ifdef MODE_DMP
        printf("Configuring for DMP mode\r\n");
        
        // Initialize DMP
        uint8_t devStatus = mpu.dmpInitialize();
        
        // Make sure it worked
        if (devStatus == 0) {
            // Calibrate sensors
            calibrateMPU();
            
            // Turn on the DMP
            printf("Enabling DMP...\r\n");
            mpu.setDMPEnabled(true);
            
            // Get expected DMP packet size
            packetSize = mpu.dmpGetFIFOPacketSize();
            dmpReady = true;
            printf("DMP ready! Packet size: %d\r\n", packetSize);
        } else {
            // ERROR!
            printf("DMP Initialization failed (code %d)\r\n", devStatus);
            while (true) {
                ThisThread::sleep_for(1s);
            }
        }
    #endif
}

int main() {
    // Set up serial communication
    printf("MPU6050 Simple Test\r\n");
    
    // Initialize MPU6050
    setupMPU();
    
    // Start timer for sample rate control
    sampleTimer.start();
    
    // Main loop
    while (true) {
        // Only update at the specified interval
        if (chrono::duration_cast<chrono::milliseconds>(sampleTimer.elapsed_time()).count() < PRINT_INTERVAL_MS) {
            ThisThread::sleep_for(5ms);
            continue;
        }
        
        // Reset timer for next interval
        sampleTimer.reset();
        
        // Read and print data based on selected mode
        #ifdef MODE_RAW_SENSORS
            // Read raw sensor data
            int16_t ax, ay, az, gx, gy, gz;
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            
            // Convert to meaningful units
            const float accel_scale = 16384.0f; // LSB/g for +/-2g range
            const float gyro_scale = 16.4f;     // LSB/(deg/s) for +/-2000 deg/s range
            
            float accel_x = ax / accel_scale;
            float accel_y = ay / accel_scale;
            float accel_z = az / accel_scale;
            float gyro_x = gx / gyro_scale;
            float gyro_y = gy / gyro_scale;
            float gyro_z = gz / gyro_scale;
            
            // Calculate Euler angles from accelerometer (pitch and roll only)
            float roll = atan2(accel_y, accel_z) * 180.0f / M_PI;
            float pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * 180.0f / M_PI;
            
            // Print raw values and calculated angles
            printf("Raw: ax=%.2f ay=%.2f az=%.2f  gx=%.2f gy=%.2f gz=%.2f\r\n", 
                   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
            printf("Angles: roll=%.2f pitch=%.2f\r\n", roll, pitch);
        #endif
        
        #ifdef MODE_FIFO
            // Check FIFO count
            uint16_t fifoCount = mpu.getFIFOCount();
            
            // Check for overflow
            if (fifoCount >= 1024) {
                printf("FIFO overflow! Resetting...\r\n");
                mpu.resetFIFO();
            } else if (fifoCount >= 12) { // We need at least accel (6) + gyro (6) = 12 bytes
                // Read a packet from FIFO
                mpu.getFIFOBytes(fifoBuffer, 12);
                
                // Extract data
                int16_t ax = (fifoBuffer[0] << 8) | fifoBuffer[1];
                int16_t ay = (fifoBuffer[2] << 8) | fifoBuffer[3];
                int16_t az = (fifoBuffer[4] << 8) | fifoBuffer[5];
                int16_t gx = (fifoBuffer[6] << 8) | fifoBuffer[7];
                int16_t gy = (fifoBuffer[8] << 8) | fifoBuffer[9];
                int16_t gz = (fifoBuffer[10] << 8) | fifoBuffer[11];
                
                // Convert to meaningful units
                const float accel_scale = 16384.0f;
                const float gyro_scale = 16.4f;
                
                float accel_x = ax / accel_scale;
                float accel_y = ay / accel_scale;
                float accel_z = az / accel_scale;
                float gyro_x = gx / gyro_scale;
                float gyro_y = gy / gyro_scale;
                float gyro_z = gz / gyro_scale;
                
                // Calculate Euler angles from accelerometer (pitch and roll only)
                float roll = atan2(accel_y, accel_z) * 180.0f / M_PI;
                float pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * 180.0f / M_PI;
                
                printf("FIFO: ax=%.2f ay=%.2f az=%.2f  gx=%.2f gy=%.2f gz=%.2f\r\n", 
                       accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
                printf("Angles: roll=%.2f pitch=%.2f\r\n", roll, pitch);
            }
        #endif
        
        #ifdef MODE_DMP
            // If DMP initialization failed, just return
            if (!dmpReady) continue;
            
            // Get current FIFO count
            fifoCount = mpu.getFIFOCount();
            
            // Check for overflow (very common issue)
            if (fifoCount >= 1024) {
                mpu.resetFIFO();
                printf("FIFO overflow!\r\n");
                continue;
            }
            
            // Wait for correct available data length
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
            
            // Read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // Get Quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            
            // Get Euler angles (directly from quaternion)
            float euler[3]; // [psi, theta, phi] Euler angle container
            mpu.dmpGetEuler(euler, &q);
            
            // Convert to degrees
            float psi = euler[0] * 180/M_PI;
            float theta = euler[1] * 180/M_PI;
            float phi = euler[2] * 180/M_PI;
            
            // Print Euler angles
            printf("Euler: Psi=%.2f Theta=%.2f Phi=%.2f\r\n", psi, theta, phi);
            
            // Get gravity vector
            mpu.dmpGetGravity(&gravity, &q);
            
            // Get Yaw/Pitch/Roll (using gravity and quaternion)
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            // Convert to degrees
            float yaw = ypr[0] * 180/M_PI;
            float pitch = ypr[1] * 180/M_PI;
            float roll = ypr[2] * 180/M_PI;
            
            // Print YPR angles
            printf("YPR: Yaw=%.2f Pitch=%.2f Roll=%.2f\r\n", yaw, pitch, roll);
            
            // Get accelerometer data
            VectorInt16 accel;
            mpu.dmpGetAccel(&accel, fifoBuffer);
            
            // Convert to g units (assuming ±2g range → 16384 LSB/g)
            const float accel_scale = 16384.0f;
            float accel_x = accel.x / accel_scale;
            float accel_y = accel.y / accel_scale;
            float accel_z = accel.z / accel_scale;
            
            // Print accelerometer values
            printf("Accel: X=%.2f Y=%.2f Z=%.2f (g)\r\n", accel_x, accel_y, accel_z);
            
            // Optional: Get linear acceleration (acceleration without gravity)
            VectorInt16 linearAccel;
            mpu.dmpGetLinearAccel(&linearAccel, &accel, &gravity);
            
            // Convert to g units
            float linear_x = linearAccel.x / accel_scale;
            float linear_y = linearAccel.y / accel_scale;
            float linear_z = linearAccel.z / accel_scale;
            
            // Print linear acceleration (without gravity)
            printf("Linear: X=%.2f Y=%.2f Z=%.2f (g)\r\n", linear_x, linear_y, linear_z);
            
        #endif
        
        // Toggle LED to show activity
        blinkState = !blinkState;
        led = blinkState;
    }
}