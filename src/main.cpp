#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "ports_v5.0.h"
#include <mbed.h>

// Define which outputs to use
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL

// Serial communication
static BufferedSerial serial(USBTX, USBRX, 115200);

// LED for activity indication
static DigitalOut led(PIN_LED_RED);
bool blinkState = false;

// MPU6050 with default I2C address
MPU6050 mpu;

// Interrupt pin setup (connect MPU6050 INT pin to this STM32 pin)
// Modify this to match your specific hardware connection
static InterruptIn mpuInterrupt(PC_13); // Change to your INT pin connection

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Volatile flag for interrupt handling
volatile bool mpuDataReady = false;

// Interrupt handler
void dmpDataReady() {
    mpuDataReady = true;
}

// Function to get current FIFO packet
bool getMotionData() {
    // If data not ready, return false
    if (!mpuDataReady && !mpuInterrupt.read()) return false;
    
    mpuDataReady = false;
    
    // Get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();
    
    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
    // Check for overflow (very common problem)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // Reset FIFO
        mpu.resetFIFO();
        printf("FIFO overflow! Reset\r\n");
        return false;
    }
    
    // Check if data is ready
    if (mpuIntStatus & 0x02) {
        // Wait for correct FIFO size
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        // Read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // Track FIFO count for multi-packet handling
        fifoCount -= packetSize;
        
        return true;
    }
    
    return false;
}

int main() {
    // Initialize serial
    printf("MPU6050 DMP Example for STM32/MbedOS\r\n");
    
    // Initialize MPU6050
    printf("Initializing I2C devices...\r\n");
    mpu.initialize();
    
    // Verify connection
    printf("Testing device connections...\r\n");
    if (mpu.testConnection()) {
        printf("MPU6050 connection successful\r\n");
    } else {
        printf("MPU6050 connection failed\r\n");
        while (true) {
            ThisThread::sleep_for(1s);
        }
    }
    
    // Initialize DMP
    printf("Initializing DMP...\r\n");
    devStatus = mpu.dmpInitialize();
    
    // Supply your own gyro offsets here, scaled for min sensitivity
    // These are example values - you should determine your own
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    
    // Make sure initialization worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibrate device - this is equivalent to Arduino's CalibrateAccel/CalibrateGyro
        printf("Calibrating...\r\n");
        
        // Auto-calibration code - collect multiple samples
        const int calibrationSamples = 12; // 6 for each axis
        int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
        int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
        
        // Collect calibration data
        for (int i = 0; i < calibrationSamples; i++) {
            int16_t ax, ay, az, gx, gy, gz;
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            ax_offset += ax;
            ay_offset += ay;
            az_offset += (az - 16384); // Remove 1g gravity from Z
            gx_offset += gx;
            gy_offset += gy;
            gz_offset += gz;
            ThisThread::sleep_for(50ms);
        }
        
        // Calculate average offsets
        ax_offset /= calibrationSamples;
        ay_offset /= calibrationSamples;
        az_offset /= calibrationSamples;
        gx_offset /= calibrationSamples;
        gy_offset /= calibrationSamples;
        gz_offset /= calibrationSamples;
        
        // Set offsets
        mpu.setXAccelOffset(-ax_offset);
        mpu.setYAccelOffset(-ay_offset);
        mpu.setZAccelOffset(-az_offset);
        mpu.setXGyroOffset(-gx_offset);
        mpu.setYGyroOffset(-gy_offset);
        mpu.setZGyroOffset(-gz_offset);
        
        printf("Offsets: AX=%d AY=%d AZ=%d GX=%d GY=%d GZ=%d\r\n", 
               -ax_offset, -ay_offset, -az_offset, -gx_offset, -gy_offset, -gz_offset);
        
        // Turn on the DMP
        printf("Enabling DMP...\r\n");
        mpu.setDMPEnabled(true);
        
        // Setup interrupt
        mpuInterrupt.rise(&dmpDataReady);
        mpuIntStatus = mpu.getIntStatus();
        
        // Set DMP ready flag
        printf("DMP ready! Waiting for first interrupt...\r\n");
        dmpReady = true;
        
        // Get expected DMP packet size
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        // Configure DMP rate to reduce overflow risk
        mpu.setRate(4); // 40Hz output rate (200 / (1 + 4))
        
        // Reset FIFO before starting
        mpu.resetFIFO();
    } else {
        // ERROR!
        printf("DMP Initialization failed (code %d)\r\n", devStatus);
        while (true) {
            ThisThread::sleep_for(1s);
        }
    }
    
    // Main loop
    while (true) {
        // If DMP not ready, wait
        if (!dmpReady) {
            ThisThread::sleep_for(10ms);
            continue;
        }
        
        // Try to get motion data
        if (getMotionData()) {
            // Process motion data based on selected output format
            
            #ifdef OUTPUT_READABLE_QUATERNION
                // Display quaternion values
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                printf("quat\t%.4f\t%.4f\t%.4f\t%.4f\r\n", q.w, q.x, q.y, q.z);
            #endif
            
            #ifdef OUTPUT_READABLE_EULER
                // Display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetEuler(euler, &q);
                printf("euler\t%.2f\t%.2f\t%.2f\r\n", 
                       euler[0] * 180/M_PI, euler[1] * 180/M_PI, euler[2] * 180/M_PI);
            #endif
            
            #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // Display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                printf("ypr\t%.2f\t%.2f\t%.2f\r\n", 
                       ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
            #endif
            
            #ifdef OUTPUT_READABLE_REALACCEL
                // Display real acceleration without gravity
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                printf("areal\t%d\t%d\t%d\r\n", aaReal.x, aaReal.y, aaReal.z);
            #endif
            
            #ifdef OUTPUT_READABLE_WORLDACCEL
                // Display world-frame acceleration without gravity
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                printf("aworld\t%d\t%d\t%d\r\n", aaWorld.x, aaWorld.y, aaWorld.z);
            #endif
            
            // Blink LED to indicate activity
            blinkState = !blinkState;
            led = blinkState;
        }
        
        // Short delay to prevent too frequent polling
        ThisThread::sleep_for(5ms);
    }
}