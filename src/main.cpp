#include "mbed.h"
#include "rtos/Thread.h"
// #include "drivers/stm32f429i_discovery_lcd.h"
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/stm32f429i_discovery_ts.h"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

volatile int TouchCount = 0;

struct GyroData {
    float x;
    float y;
    float z;

    GyroData(float x, float y, float z) : x(x), y(y), z(z) {}
};

using GyroDataSet = std::vector<GyroData>;

GyroDataSet data1;
GyroDataSet data2;

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

std::string UnlockStatus = "...";

float similarity = 0.00;

volatile bool isRecording = false;

// SPI模式读取陀螺仪
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

// 初始化触摸屏
TS_StateTypeDef ts_state;

// 初始化LCD
LCD_DISCO_F429ZI lcd;

// 初始化LED引脚
DigitalOut led(LED1);

bool reset = false;
// 初始化用户按钮（用于重置状态）
DigitalIn userButton(USER_BUTTON);
// 用户按钮中断
InterruptIn buttonInterrupt(USER_BUTTON);

void buttonPressedCallback()
{
    reset = true;
}

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

bool isTouchEnded() {
    TS_StateTypeDef currentState;
    ThisThread::sleep_for(50ms); // 等待一段时间，以确保手指离开触摸屏
    BSP_TS_GetState(&currentState);
    return !currentState.TouchDetected;
}

int main() {
    buttonInterrupt.rise(&buttonPressedCallback); // // 设置上升沿中断回调函数

    // int16_t raw_gz = gyro_thread();
    // 初始化LCD
    lcd.Clear(LCD_COLOR_WHITE);
    lcd.SetBackColor(LCD_COLOR_WHITE);
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.SetFont(&Font20);

    // 转换整数和浮点数为字符串
    char buffer1[32];
    char buffer2[32];
    char buffer3[32];
    // sprintf(buffer, "Integer: %d", integer_value);

    // 在屏幕上显示整数
    uint16_t x = 20;
    uint16_t y = 0;
    // lcd.DisplayStringAt(x, y, (uint8_t *)buffer, LEFT_MODE);
    // 转换浮点数为字符串（保留两位小数）
    // 小数：%2.f
    // sprintf(buffer, "Float: %2.f", similarity);

    // 在屏幕上显示浮点数
    // y += lcd.GetFont()->Height;
    // lcd.DisplayStringAt(x, y, (uint8_t *)buffer, LEFT_MODE);

    // Gyro thread
    // 创建线程对象
    // Thread thread;
    // 启动线程，执行external_function
    // thread.start(gyro_thread);

    // Initialize LCD and touchscreen
    // BSP_LCD_Init();
    // BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
    BSP_TS_Init(lcd.GetXSize(), lcd.GetYSize());

    while (1) {
        if (reset == true) {
            isRecording = false;
            TouchCount = 0; // 当按键被按下时，将计数变量count设为0
            similarity = 0.00;
            GyroDataSet data1;
            GyroDataSet data2;
            UnlockStatus = "...";
            reset = false;
        }

        sprintf(buffer1, "TouchCount: %d", TouchCount);
        sprintf(buffer2, "Similarity: %2.f", similarity);
        sprintf(buffer3, "Unlocked: %s", UnlockStatus.c_str());
        BSP_TS_GetState(&ts_state);

        if (TouchCount < 4 && ts_state.TouchDetected && isTouchEnded()) {
            isRecording = !isRecording; // Switcher
            TouchCount = TouchCount + 1;
        }

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

        if (TouchCount == 4) {
            similarity = calculateDTWDistance(data1, data2);
            if (similarity < 60) {
                UnlockStatus = "YES";
            } else {
                UnlockStatus = "NO ";
            }
        }

        // ThisThread::sleep_for(1s);
        lcd.DisplayStringAt(x, 10, (uint8_t *)buffer1, LEFT_MODE);
        lcd.DisplayStringAt(x, 35, (uint8_t *)buffer2, LEFT_MODE);
        lcd.DisplayStringAt(x, 60, (uint8_t *)buffer3, LEFT_MODE);
    }

}