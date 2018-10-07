/*

Analog input, analog output
Reads an analog input pin, uses voltage to calculate pressure,
pressure is converted to velocity using v=k*sqrt(P), velocity
is converted to flow rate, flow rate is integrated over time to
determine total volume passed through the spirometer.
Total time is recorded by pressing appropriate buttons on the device.
Results are printed to an LCD screen.

Code adapted from: http://www.instructables.com/id/Low-Cost-Spirometer/

*/
// include the library code
#include "Arduino.h"
#include <ResponsiveAnalogRead.h>
#include "SPI.h"
#include "Wire.h"
#include <U8g2lib.h>

uint8_t ANALOG_IN = A0; // Analog input pin, connected to pressure sensor
uint8_t BUTTON = D5; // Button

ResponsiveAnalogRead analog(ANALOG_IN, true, 0.001);

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//Variables to change
int sensorReading = 0; // Voltage read from pressure sensor (in bits, 0 to 1023)
volatile double pressure_pa = 0; // Pressure converted to Pa
volatile double massFlow = 0; // Mass flow rate calculated from pressure
volatile double volFlow = 0; // Calculated from mass flow rate
volatile double volume = 0; // Integral of flow rate over time

int calibration = 0; // floating voltage
volatile unsigned long time = 0;
volatile float dt = 0;
int idx = 0; // graph pixel index

//Constants
const double rho = 1.225; // Density of air in kg/m3
const double area_1 = 0.000326726; // Surface area in m2
const double area_2 = 0.00002827; // Surface area in m2


enum State {
    IDLE,
    EXHALE,
    RESULTS
};

int tmp[5000];

State state = IDLE;
int paCount = 0;


void resetReadings() {
    time = 0;
    state = IDLE;
    dt = 0;
    volume = 0;
    tmp[5000] = {};
    idx = 0;
}

ICACHE_RAM_ATTR void timer0_ISR(void) {
    analog.update(analogRead(ANALOG_IN) - calibration);
    sensorReading = analog.getValue(); // Voltage read in (0 to 1023)
    sensorReading = sensorReading < 0 ? 0 : sensorReading;
    pressure_pa = sensorReading * 9.77517106549; // 1023 == 10kPa, therefore 9.77517106549 Pa/unit in sensorReading
    massFlow = 1000 * sqrt((pressure_pa * 2 * rho) /
                           ((1 / (pow(area_2, 2))) - (1 / (pow(area_1, 2))))); // Mass flow of air
    volFlow = massFlow / rho;

    switch (state) {
        case IDLE:
            if (pressure_pa > 1) {
                paCount++;
            }
            if (paCount > 50) {
                state = EXHALE;
            }
            break;
        case EXHALE:
            volume = volFlow * dt + volume; // Total volume (essentially integrated over time)
            dt = 0.001;
            time++;
            if (pressure_pa < 1) {
                paCount--;
            }
            if (paCount < 0) {
                state = RESULTS;
            }
            tmp[idx] = sensorReading;
            idx++;
            break;
        case RESULTS:
            break;
    }
    timer0_write(ESP.getCycleCount() + 80000); // 80MHz == 1sec
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON, INPUT);
    Wire.begin();
    Serial.println("Init");
    for (int i = 0; i < 100; i++) {
        calibration += analogRead(A0);
        delay(1);
    }
    calibration = (int) (calibration / 100.0);
    u8g2.begin();
    u8g2.clearDisplay();
    attachInterrupt(BUTTON, resetReadings, RISING);
    noInterrupts();
    timer0_isr_init();
    timer0_attachInterrupt(timer0_ISR);
    timer0_write(ESP.getCycleCount() + 80000); // 80MHz == 1sec
    interrupts();
}


void loop() {
    u8g2.firstPage();
    do {
        u8g2.setFont(u8g2_font_4x6_tf);
        const char *VOLTAGE = "R: ";
        const char *PRESSURE = "P: ";
        const char *MFLOW = "MF: ";
        const char *VFLOW = "VF: ";
        const char *STATE = "S: ";
        const char *TIME = "T: ";
        const char *VOLUME = "L/s: ";

        u8g2.drawStr(0, 6, VOLTAGE);
        u8g2.drawStr(u8g2.getStrWidth(VOLTAGE), 6, String(sensorReading).c_str());

        u8g2.drawStr(64, 6, PRESSURE);
        u8g2.drawStr(u8g2.getStrWidth(PRESSURE) + 64, 6, String(pressure_pa).c_str());

        u8g2.drawStr(0, 12, MFLOW);
        u8g2.drawStr(u8g2.getStrWidth(MFLOW), 12, String(massFlow).c_str());

        u8g2.drawStr(64, 12, VFLOW);
        u8g2.drawStr(u8g2.getStrWidth(VFLOW) + 64, 12, String(volFlow).c_str());


        u8g2.drawStr(0, 64, STATE);
        u8g2.drawStr(u8g2.getStrWidth(STATE), 64, String(state).c_str());

        u8g2.drawStr(12, 64, VOLUME);
        u8g2.drawStr(u8g2.getStrWidth(VOLUME) + 12, 64, String(volume).c_str());

        u8g2.drawStr(64, 64, TIME);
        u8g2.drawStr(u8g2.getStrWidth(TIME) + 64, 64, String((float) time / 1000.0).c_str());

        if (state != IDLE) {
            int interval = idx / 128;
            for (int i = 0; i < 128; i++) {
                u8g2.drawPixel(i, (u8g2_uint_t) map(tmp[i * interval], 0, 1023, 56, 8));
            }
        }

    } while (u8g2.nextPage());
}
