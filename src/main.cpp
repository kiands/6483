#include "mbed.h"
#include "rtos/Thread.h"
// #include "drivers/stm32f429i_discovery_lcd.h"
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/stm32f429i_discovery_ts.h"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

/* This defines the recording/calculating/comparaing status. Increase by 1 after each TouchScreen event, max is 4.
0 = initial status, not recording.
1 = recording round 1.
2 = pause, not recording.
3 = recording round 2.
4 = calculate, not recording.
*/
volatile int TouchCount = 0;

// Indicator that will be displayed to inform user about the unlock status. `...` is the original status.
std::string UnlockStatus = "...";
// The initial threshold for define success or fail. In the main function, threshold will be dynamic according to the size of data1.
int threshold = 0;
// Similarity, will be compared with threshold.
float similarity = 0.00;

volatile bool isRecording = false;

// The data structure used to store x/y/z axis data.
struct GyroData {
    float x;
    float y;
    float z;

    GyroData(float x, float y, float z) : x(x), y(y), z(z) {}
};

using GyroDataSet = std::vector<GyroData>;

// Initialize 2 instances. `data1` is used to store the reference gesture, `data2` is used to store the gesture that needs to be compared.
GyroDataSet data1;
GyroDataSet data2;

// The two functions are used to calculate the original similarity of data1 and data2.
float euclideanDistance(const GyroData& a, const GyroData& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

float calculateDTWDistance(const GyroDataSet& a, const GyroDataSet& b) {
    int n = a.size();
    int m = b.size();
    std::vector<std::vector<float>> dtw(n + 1, std::vector<float>(m + 1, std::numeric_limits<float>::infinity()));
    dtw[0][0] = 0;

    for (int i = 1; i <= n; ++i) {
        for (int j = 1; j <= m; ++j) {
                        float cost = euclideanDistance(a[i - 1], b[j - 1]);
            dtw[i][j] = cost + std::min({dtw[i - 1][j], dtw[i][j - 1], dtw[i - 1][j - 1]});
        }
    }

    return dtw[n][m];
}

// Initialize gyroscope with SPI mode.
SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel); // mosi, miso, sclk, cs

#define OUT_X_L 0x28
//register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG1 0x20
//configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
//register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)
#define CTRL_REG4 0x23
//configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

#define SPI_FLAG 1

uint8_t write_buf[32];
uint8_t read_buf[32];

EventFlags flags;
//The spi.transfer function requires that the callback
//provided to it takes an int parameter
void spi_cb(int event){
  flags.set(SPI_FLAG);
};

GyroData gyro_thread() {
    // Setup the spi for 8 bit data, high steady state clock,
    // second edge capture, with a 1MHz clock rate
    spi.format(8,3);
    spi.frequency(1'000'000);

    write_buf[0]=CTRL_REG1;
    write_buf[1]=CTRL_REG1_CONFIG;
    spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    write_buf[0]=CTRL_REG4;
    write_buf[1]=CTRL_REG4_CONFIG;
    spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    volatile int16_t raw_gx,raw_gy,raw_gz;
    volatile float gx, gy, gz;
    //reading the status register. bit 4 of the status register
    //is 1 when a new set of samples is ready
    write_buf[0]=0x27 | 0x80;
    do{
    spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);
    }while((read_buf[1]&0b0000'1000)==0);

    //prepare the write buffer to trigger a sequential read
    write_buf[0]=OUT_X_L|0x80|0x40;

    //start sequential sample reading
    spi.transfer(write_buf,7,read_buf,8,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    //read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
    //Put the high and low bytes in the correct order lowB,Highb -> HighB,LowB
    raw_gx=( ( (uint16_t)read_buf[2] ) <<8 ) | ( (uint16_t)read_buf[1] );
    raw_gy=( ( (uint16_t)read_buf[4] ) <<8 ) | ( (uint16_t)read_buf[3] );
    raw_gz=( ( (uint16_t)read_buf[6] ) <<8 ) | ( (uint16_t)read_buf[5] );

    // printf("RAW|\tgx: %d \t gy: %d \t gz: %d\n",raw_gx,raw_gy,raw_gz);

    gx=((float)raw_gx)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
    gy=((float)raw_gy)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
    gz=((float)raw_gz)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
    GyroData gyroSample(gx, gy, gz);
    // data2.push_back(gyroSample);
    // ThisThread::sleep_for(100);
    return gyroSample;
}

// Peripherals used to achieve better UX.
DigitalOut led(LED1); // Initialize LED. When recording, it will be on, else will be off.
DigitalIn userButton(USER_BUTTON); // Initialize UserButton for resetting the status
InterruptIn buttonInterrupt(USER_BUTTON); // UserButton interrupt.
TS_StateTypeDef ts_state; // Initialize TouchScreen
LCD_DISCO_F429ZI lcd; // Initialize LCD

bool reset = false; // Reset toggle. The UserButton interrupt will set this to be true and the reset related code will take effect in main function.
bool retry = false; // Unsuccessful unlock retry toggle.

// The UserButton pressed event callback.
void buttonPressedCallback()
{
    reset = true;
}

// TouchScreen event filter to avoid multiple counts, very important
bool isTouchEnded() {
    TS_StateTypeDef currentState;
    ThisThread::sleep_for(50ms); // Wait some time to make sure that finger has left.
    BSP_TS_GetState(&currentState);
    return !currentState.TouchDetected;
}

// finally, main function starts!
int main() {
    buttonInterrupt.rise(&buttonPressedCallback); // Set the rising edge interrupt callback function.

    // int16_t raw_gz = gyro_thread();
    // Initialize LCD display.
    lcd.Clear(LCD_COLOR_WHITE);
    lcd.SetBackColor(LCD_COLOR_WHITE);
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.SetFont(&Font20);

    // Convert contents we want to display to string.
    char buffer1[32];
    char buffer2[32];
    char buffer3[32];
    char buffer4[32];
    char buffer5[32];
    char buffer6[32];

    uint16_t x = 20;
    uint16_t y = 0;

    // Gyro thread
    // 创建线程对象
    // Thread thread;
    // 启动线程，执行external_function
    // thread.start(gyro_thread);

    BSP_TS_Init(lcd.GetXSize(), lcd.GetYSize());

    // Indicator Variables
    int int_size_data1 = 0; // Length of data1.
    float ratio = 0.00; // The length ratio of data2 when compared to data1.

    while (1) {
        if (reset == true && retry == false) { // When we need to reset and don't deed to retry
            lcd.Clear(LCD_COLOR_WHITE); // clear LCD
            data1.clear();
            data2.clear();
            isRecording = false;
            TouchCount = 0; // for this situation, TouchCount should be 0
            similarity = 0.00;
            int_size_data1 = 0;
            ratio = 0.00;
            UnlockStatus = "...";
            reset = false;
            threshold = 0;
        } else if (reset == true && retry == true){
            lcd.Clear(LCD_COLOR_WHITE); // clear LCD
            // Reserve data 1 and clear data2.
            data2.clear();
            isRecording = false;
            TouchCount = 2; // Directly jump to 2 (before round2 start).
            similarity = 0.00;
            ratio = 0.00;
            UnlockStatus = "...";
            reset = false;
            retry = false;
        }

        sprintf(buffer1, "TouchCount: %d", TouchCount);
        sprintf(buffer2, "Similarity: %d", static_cast<int>(similarity));
        sprintf(buffer3, "Unlocked: %s", UnlockStatus.c_str());
        sprintf(buffer4, "Len_data1: %d", int_size_data1);
        sprintf(buffer5, "Ratio: %.2f", ratio);
        sprintf(buffer6, "Threshold: %d", threshold);
        BSP_TS_GetState(&ts_state);

        // This part prevents TouchCount from increasing after reaching 4.
        if (TouchCount < 4 && ts_state.TouchDetected && isTouchEnded()) {
            isRecording = !isRecording; // Recording switcher.
            TouchCount = TouchCount + 1;
        }

        // This part defines whether push data into data1 or data2.
        if (isRecording) {
            led = 1;
            if (TouchCount == 1) {
                data1.push_back(gyro_thread());
            } else if (TouchCount == 3) {
                data2.push_back(gyro_thread());
            }
            ThisThread::sleep_for(25);
        } else {
            led = 0;
        }

        // The decisions will be made in this part.
        if (TouchCount == 4) {
            similarity = calculateDTWDistance(data1, data2);
            size_t size_data1 = data1.size(); // Calculate size of data1
            size_t size_data2 = data2.size(); // Calculate size of data2
            int_size_data1 = static_cast<int>(size_data1);
            ratio = static_cast<float>(size_data2) / static_cast<float>(size_data1);
            // Standard: length of data1 = 100, threshold is 90.
            threshold = static_cast<int>((static_cast<float>(int_size_data1) / 100.0f) * 90.0f);
            if (ratio < 0.75 || ratio > 1.25) { // When data1 and data2 have too big length difference, report error and set the process to retry.
                UnlockStatus = "ERR";
                retry = true;
            } else {
                if (similarity < threshold) { // When `similarity`(euclidean distance) is within threshold, report YES.
                    UnlockStatus = "YES";
                } else { // When `similarity`(euclidean distance) exceeds threshold, report YES.
                    UnlockStatus = "NO ";
                    retry = true;
                }
            }
        }

        lcd.DisplayStringAt(x, 10, (uint8_t *)buffer1, LEFT_MODE);
        lcd.DisplayStringAt(x, 35, (uint8_t *)buffer2, LEFT_MODE);
        lcd.DisplayStringAt(x, 60, (uint8_t *)buffer3, LEFT_MODE);
        lcd.DisplayStringAt(x, 85, (uint8_t *)buffer4, LEFT_MODE);
        lcd.DisplayStringAt(x, 110, (uint8_t *)buffer5, LEFT_MODE);
        lcd.DisplayStringAt(x, 135, (uint8_t *)buffer6, LEFT_MODE);
    }

}