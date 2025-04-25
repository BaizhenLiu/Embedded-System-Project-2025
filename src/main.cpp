#include "mbed.h"
#include "arm_math.h" 
#include <cmath>
#include <chrono>
using namespace std::chrono_literals;


I2C i2c(PB_11, PB_10);  // I2C2: SDA = PB11, SCL = PB10
// 设置I2C SDA和SCL引脚（用于数据传输）

BufferedSerial serial_port(USBTX, USBRX, 115200);
//串口配置，用于将数据发送到PC或其他串口设备
FileHandle *mbed::mbed_override_console(int) {
    return &serial_port;  
}//如果是mac就用这段代码

// LSM6DSL address (0x6A in datasheet, shifted left for 8-bit format)
#define LSM6DSL_ADDR (0x6A << 1)  // Equals 0xD4

//48 and 49th pages in LSM6DSL datasheet XL是加速器，G是陀螺仪
#define WHO_AM_I    0x0F  // ID register - should return 0x6A
#define CTRL1_XL    0x10  // Accelerometer control register to configure range 加速度计控制寄存器
#define CTRL2_G     0x11  // Gyroscope control register to configure range 陀螺仪控制寄存器
#define OUTX_L_XL   0x28  // XL X-axis (low byte) 加速度计X轴数据寄存器低字节
#define OUTX_H_XL   0x29  // XL X-axis (high byte) 加速度计X轴数据寄存器高字节
#define OUTY_L_XL   0x2A  // XL Y-axis (low byte) 加速度计Y轴数据寄存器低字节
#define OUTY_H_XL   0x2B  // XL Y-axis (high byte)
#define OUTZ_L_XL   0x2C  // XL Z-axis (low byte) 加速度计Z轴数据寄存器低字节
#define OUTZ_H_XL   0x2D  // XL Z-axis (high byte)
#define OUTX_L_G    0x22  // Gyro X-axis (low byte) 
#define OUTX_H_G    0x23  // Gyro X-axis (high byte)
#define OUTY_L_G    0x24  // Gyro Y-axis (low byte)
#define OUTY_H_G    0x25  // Gyro Y-axis (high byte)
#define OUTZ_L_G    0x26  // Gyro Z-axis (low byte)
#define OUTZ_H_G    0x27  // Gyro Z-axis (high byte)
//FFT
#define FFT_SIZE 256
// #define SAMPLE_RATE 10000.0f
#define SAMPLE_RATE 104.0f 

float32_t input_acc[FFT_SIZE];
float32_t input_gyro[FFT_SIZE];

static float32_t fft_out[FFT_SIZE];        // 中间复数输出  
static float32_t mag_acc[FFT_SIZE/2];      // 加速度 FFT 幅值  
static float32_t mag_gyro[FFT_SIZE/2]; 

arm_rfft_fast_instance_f32 FFT_Instance;

// Write a value to a register
void write_register(uint8_t reg, uint8_t value) {
    char data[2] = {(char)reg, (char)value};
    i2c.write(LSM6DSL_ADDR, data, 2);
}

// Read a value from a register
uint8_t read_register(uint8_t reg) {
    char data = reg;
    i2c.write(LSM6DSL_ADDR, &data, 1, true); // No stop
    i2c.read(LSM6DSL_ADDR, &data, 1);
    return (uint8_t)data;
}

// Read a 16-bit value (combines low and high byte registers)
int16_t read_16bit_value(uint8_t low_reg, uint8_t high_reg) {
    // Read low byte
    char low_byte = read_register(low_reg);
    
    // Read high byte
    char high_byte = read_register(high_reg);
    
    // Combine the bytes (little-endian: low byte first)
    return (high_byte << 8) | low_byte;
}
//高位低位合并获得16位数据

//init_I2C
void init_I2C() {
    i2c.frequency(400000); // 设置 I2C 频率为 400kHz
}

// 检查传感器是否连接
void checkconnection(){
    uint8_t id = read_register(WHO_AM_I);
    if (id != 0x6A) {
        printf("Error: LSM6DSL sensor not found!\r\n");
        while (1); // 停止运行
    }
}

void configure_sensor() {
    // 配置加速度计和陀螺仪的采样频率与量程
    write_register(CTRL1_XL, 0x40);  // 加速度计配置
    printf("Accelerometer configured: 104 Hz, ±2g range\r\n");
    write_register(CTRL2_G, 0x40);   // 陀螺仪配置
    printf("Gyroscope configured: 104 Hz, ±250 dps range\r\n");
}
//将没有实际物理意义的数值转换为真实的物理数值
const float ACC_SENSITIVITY = 0.061f;  // mg/LSB for ±2g range
const float GYRO_SENSITIVITY = 8.75f;  // mdps/LSB for ±250 dps range

float transformAccelerometer(int16_t acc_raw){
    float acc_g=0;
    acc_g=acc_raw * ACC_SENSITIVITY / 1000.0f;
    return acc_g;
}
float tranformGyroscope(int16_t gyro_raw){
    float gyro_dps;
    gyro_dps=gyro_raw * GYRO_SENSITIVITY / 1000.0f;
    return gyro_dps;
}
//进行FFT变换
// void run_fft() {
//     arm_rfft_fast_f32(&FFT_Instance, input_fft, fft_out, 0);  
//     arm_cmplx_mag_f32(fft_out, magnitude, FFT_SIZE / 2);      
// }

// void show_results() {
//     float resolution = SAMPLE_RATE / FFT_SIZE;  

//     printf("Bin\tFreq (Hz)\tMagnitude\n");
//     for (int i = 0; i < 28; i++) {
//         printf(" %d\t%.2f\t\t%.4f\n", i, i * resolution, magnitude[i]);
//     }

//     uint32_t max_index = 0;
//     float32_t max_val = 0.0f;
//     arm_max_f32(magnitude, FFT_SIZE/2, &max_val, &max_index);  

//     printf("Peak Frequency: %.2f Hz\n\n", max_index * resolution);
//     printf("Max magnitude: %.1f at bin %lu (%.2f Hz)\r\n", 
//         max_val, max_index, max_index * resolution);  
// }
void computeFFT(const float32_t input[], float32_t mag_out[]) {
    // 实时执行 FFT
    arm_rfft_fast_f32(&FFT_Instance, const_cast<float32_t*>(input), fft_out, 0);
    // 复数 -> 幅值
    arm_cmplx_mag_f32(fft_out, mag_out, FFT_SIZE/2);
}
struct Detection {
    bool detected;
    float freq;
    float magnitude;
};

// 震颤检测：先 FFT，再找峰，再判频段 
Detection detectTremorFFT(const float32_t acc_buf[]) {
    // 先做 FFT
    computeFFT(acc_buf, mag_acc);

    // 找峰值
    uint32_t idx; float32_t val;
    arm_max_f32(mag_acc, FFT_SIZE/2, &val, &idx);

    // 填结果
    Detection d;
    d.freq      = idx * (SAMPLE_RATE / FFT_SIZE);
    d.magnitude = val;
    d.detected  = (d.freq >= 3.0f && d.freq <= 5.0f);
    return d;
}
// 舞蹈症检测：同上 
Detection detectDyskinesiaFFT(const float32_t gyro_buf[]) {
    computeFFT(gyro_buf, mag_gyro);

    uint32_t idx; float32_t val;
    arm_max_f32(mag_gyro, FFT_SIZE/2, &val, &idx);

    Detection d;
    d.freq      = idx * (SAMPLE_RATE / FFT_SIZE);
    d.magnitude = val;
    d.detected  = (d.freq > 5.0f && d.freq <= 7.0f);
    return d;
}

int main(){

    //初始化硬件
    init_I2C();

    // 检查传感器是否连接
    checkconnection();

    // 配置传感器
    configure_sensor();
    //初始化 ARM FFT 实例
    arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE);
    int index = 0; 
    constexpr auto SAMPLE_PERIOD = 1000ms / 104;
    //精度有误差，后续需要调整

    while(1){
        //收集打印数据
        int16_t acc_x_raw = read_16bit_value(OUTX_L_XL, OUTX_H_XL);
        int16_t acc_y_raw = read_16bit_value(OUTY_L_XL, OUTY_H_XL);
        int16_t acc_z_raw = read_16bit_value(OUTZ_L_XL, OUTZ_H_XL);

        int16_t gyro_x_raw = read_16bit_value(OUTX_L_G, OUTX_H_G);
        int16_t gyro_y_raw = read_16bit_value(OUTY_L_G, OUTY_H_G);
        int16_t gyro_z_raw = read_16bit_value(OUTZ_L_G, OUTZ_H_G);

        float acc_x_g=transformAccelerometer(acc_x_raw);
        float acc_y_g=transformAccelerometer(acc_y_raw);
        float acc_z_g=transformAccelerometer(acc_z_raw);

        float gyro_x_dps=tranformGyroscope(gyro_x_raw);
        float gyro_y_dps=tranformGyroscope(gyro_y_raw);
        float gyro_z_dps=tranformGyroscope(gyro_z_raw);

        // 打印采集的数据
        printf("Accel [g]: X=%+6.3f, Y=%+6.3f, Z=%+6.3f | Gyro [dps]: X=%+7.2f, Y=%+7.2f, Z=%+7.2f\r\n", 
            acc_x_g, acc_y_g, acc_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps);
        
        //进行频谱分析
        //xyz方向数据融合
        float acc_mag = sqrt(acc_x_g * acc_x_g + acc_y_g * acc_y_g + acc_z_g * acc_z_g);
        float gyro_mag = sqrt(gyro_x_dps * gyro_x_dps + gyro_y_dps * gyro_y_dps + gyro_z_dps * gyro_z_dps);

        //填入各自缓冲
        input_acc[index]  = acc_mag;
        input_gyro[index] = gyro_mag;
        index++;

        // 如果填充满了整个 FFT 数组，执行 FFT
        if (index >= FFT_SIZE) {
            Detection tremor = detectTremorFFT(input_acc);
            Detection dysk  = detectDyskinesiaFFT(input_gyro);
            index = 0;
            if (tremor.detected) {
                printf("[Tremor Detected] Peak Frequency: %.2f Hz | Magnitude: %.4f\r\n", tremor.freq, tremor.magnitude);
            }
        
            // 根据 dysk.detected / dysk.magnitude 做陀螺指示
            if (dysk.detected) {
                printf("[Dyskinesia Detected] Peak Frequency: %.2f Hz | Magnitude: %.4f\r\n", dysk.freq, dysk.magnitude);
            }
        }
           
        // 等待下一次采样
        ThisThread::sleep_for(SAMPLE_PERIOD);

    }


}
