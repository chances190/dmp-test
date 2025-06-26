// H-Bridge
#define M1_DIR  PB_6        // M1_PHASE
#define M1_PWM  PB_5        // M1_ENABLE  TIM3_CH1
#define M1_ENCA PA_8        // TIM1_CH1
#define M1_ENCB PA_9        // TIM1_CH2

#define M2_DIR  PC_8         // M2_PHASE
#define M2_PWM  PB_7         // M2_ENABLE  TIM4_CH2
#define M2_ENCA PC_7_ALT0    // TIM8_CH2
#define M2_ENCB PC_6_ALT0    // TIM8_CH1
 
// Current Sensors
#define M1_CURR         PB_1
#define M2_CURR         PA_1
#define M1_CURR_VREF    PC_0
#define M2_CURR_VREF    PC_1

// Switch Selector
#define PIN_SELECTOR_1  PA_0
#define PIN_SELECTOR_2 	PC_13
#define PIN_SELECTOR_3 	PC_2
#define PIN_SELECTOR_4 	PC_3

// Buzzer
#define PIN_BUZZER      PB_10

// Battery
#define PIN_BATT        PC_4 

// Temperature
#define PIN_TEMP        PB_0

// RGB Led
#define PIN_LED_RED     PA_4        

// MPU
#define MPU_SCL         PB_8
#define MPU_SDA         PB_9

// NRF_RECV
#define NRF_R_MOSI  PA_7
#define NRF_R_MISO  PA_6
#define NRF_R_SCK   PA_5
#define NRF_R_CE    PC_5
#define NRF_R_CSN   PB_2
#define NRF_R_VCC   PA_12 // MUDAR PRA 3v3

// NRF_SEND
#define NRF_S_MOSI  PB_15
#define NRF_S_MISO  PB_14
#define NRF_S_SCK   PB_13
#define NRF_S_CE    PD_2
#define NRF_S_CSN   PB_12
#define NRF_S_VCC   PC_12 // MUDAR PRA 3v3

// ST_LINK
// TX      -> PA_2
// RX      -> PA_3
// SWO     -> PB_3
// NRST    -> RST
// SWD_IO  -> PA_13
// GND     -> GND
// SWD_CLK -> PA_14
// 3V3     -> VCC_3V3