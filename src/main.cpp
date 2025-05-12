#include "mbed.h"
#include "arm_math.h" 
#include <cmath>
#include <chrono>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/gap/AdvertisingDataBuilder.h"
#include "platform/Callback.h"
#include "ble/GattServer.h"

using namespace std::chrono_literals;
using namespace std::chrono;

volatile bool is_connected = false;
DigitalOut led(LED1);

BLE &ble_interface = BLE::Instance();
events::EventQueue event_queue;

const UUID tremorServiceUUID("e7810a71-73ae-499d-8c15-faa9aef0c3f2");
const UUID tremorCharUUID("befc5c1c-a5d0-42db-a6c2-c0bfa020e50d");

class GapEventHandler : public ble::Gap::EventHandler {
    void onConnectionComplete(const ble::ConnectionCompleteEvent &event) override {
        is_connected = true;
        printf("[BLE] Connected\n");
    }

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event) override {
        is_connected = false;
        printf("[BLE] Disconnected\n");

        // 断开后重新开始广播
        BLE::Instance().gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
    }
};


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
uint8_t tremor_value[2] = {0, 0};

float32_t input_acc[FFT_SIZE];
float32_t input_gyro[FFT_SIZE];

static float32_t fft_out[FFT_SIZE];        // 中间复数输出  
static float32_t mag_acc[FFT_SIZE/2];      // 加速度 FFT 幅值  
static float32_t mag_gyro[FFT_SIZE/2]; 

arm_rfft_fast_instance_f32 FFT_Instance;

uint8_t tremor_state = 0; // 0 = 正常, 1 = 抖动报警
GattCharacteristic tremorChar(
    tremorCharUUID,               // UUID
    tremor_value,                 // 初始值指针
    sizeof(tremor_value),        // 最小长度
    sizeof(tremor_value),        // 最大长度
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY |
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ,  // 权限
    nullptr,                     // 无描述符
    0,                           // 描述符数量为 0
    false                        // 不使用 variable length
);



// 修正：使用一个指向 GattCharacteristic 的指针数组
GattCharacteristic* characteristics[] = { &tremorChar };

// 创建 GattService 时传入指向 GattCharacteristic 的指针数组
GattService tremorService(tremorServiceUUID, characteristics, 1);

void update_tremor_state() {
    static int counter = 0;
    counter++;

    if (true) {
        tremor_state = (tremor_state == 0) ? 1 : 0;
        led = tremor_state;

        // 通知更新震颤状态
        ble_interface.gattServer().write(tremorChar.getValueHandle(), &tremor_state, sizeof(tremor_state));
        printf("Tremor state updated: %d\n", tremor_state);
    }
}

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(callback(&ble_interface, &BLE::processEvents));
}

void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
    if (params->error != BLE_ERROR_NONE) {
        printf("BLE init failed.\n");
        return;
    }

    // 设置 GAP 事件处理器（处理连接/断开事件）
    static GapEventHandler gap_handler;
    ble_interface.gap().setEventHandler(&gap_handler);

    // 添加自定义 GATT 服务
    ble_interface.gattServer().addService(tremorService);

    // 设置广播参数
    ble::AdvertisingParameters adv_params(
        ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
        ble::adv_interval_t(ble::millisecond_t(1000))
    );

    // 设置广播内容
    static uint8_t adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder adv_data_builder(adv_buffer);
    adv_data_builder.setFlags();
    adv_data_builder.setName("TremorBLE"); // 广播名称，在手机 BLE 工具中能看到

    ble_interface.gap().setAdvertisingParameters(ble::LEGACY_ADVERTISING_HANDLE, adv_params);
    ble_interface.gap().setAdvertisingPayload(ble::LEGACY_ADVERTISING_HANDLE, adv_data_builder.getAdvertisingData());

    // 启动广播
    ble_interface.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
    printf("BLE service started. You can now connect from phone.\n");
}



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
    write_register(CTRL1_XL, 0x40);  
    write_register(CTRL2_G, 0x40);   

    // 读取回寄存器验证是否写成功
    uint8_t xl_conf = read_register(CTRL1_XL);
    uint8_t g_conf = read_register(CTRL2_G);
    printf("CTRL1_XL = 0x%02X (expect 0x40)\r\n", xl_conf);
    printf("CTRL2_G  = 0x%02X (expect 0x40)\r\n", g_conf);
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
//函数正确，检测mag_out有数值且小于1000，并且在左右摆动时度数有明显变化，大于10,000
void computeFFT(const float32_t input[], float32_t mag_out[]) {
    // 实时执行 FFT
    arm_rfft_fast_f32(&FFT_Instance, const_cast<float32_t*>(input), fft_out, 0);
    // 复数 -> 幅值
    arm_cmplx_mag_f32(fft_out, mag_out, FFT_SIZE/2);

    // for (int i = 0; i < FFT_SIZE / 2; i++) {
    //     // 将幅值放大1000倍后转为整数
    //     int mag_int = (int)(mag_out[i] * 1000);
    
    //     // 打印带有三位小数的幅值
    //     printf("FFT Bin %d: %d.%03d\n", i, mag_int / 1000, mag_int % 1000);
    // }
}
struct Detection {
    bool detected;
    float freq;
    float magnitude;
};

Detection detectTremorFFT(const float32_t acc_buf[]) {
    // Step 1: 去除 DC 分量
    float32_t acc_buf_dcd[FFT_SIZE];
    float32_t mean = 0.0f;
    
    // 计算均值
    for (int i = 0; i < FFT_SIZE; i++) {
        mean += acc_buf[i];
    }
    mean /= FFT_SIZE;

    // 去除 DC 分量
    for (int i = 0; i < FFT_SIZE; i++) {
        acc_buf_dcd[i] = acc_buf[i] - mean;
    }

    // Step 2: 打印去除 DC 分量后的 acc_buf（可选）
    // for (int i = 0; i < 10; i++) {
    //     printf("acc_buf[%d] = %d.%03d\n", i, (int)(acc_buf_dcd[i]), (int)(acc_buf_dcd[i] * 1000) % 1000);
    // }

    // Step 3: 做 FFT
    computeFFT(acc_buf_dcd, mag_acc);

    // Step 4: 打印 FFT 结果
    // for (int i = 0; i < FFT_SIZE / 2; i++) {
    //     printf("FFT Bin %d: %d\n", i, mag_acc[i]);
    // }

    // Step 5: 找峰值
    uint32_t idx; 
    float32_t val;
    arm_max_f32(mag_acc, FFT_SIZE / 2, &val, &idx);

    // Step 6: 填结果
    Detection d;
    d.freq      = idx * (SAMPLE_RATE / FFT_SIZE);
    d.magnitude = val;

    // 设置为2-10hz
    d.detected  = (d.freq >= 5.0f && d.freq <= 10.0f); // 允许的震颤频段范围

    // Step 7: 打印调试信息
    // printf("Peak Idx : %lu\n", idx);
    // printf("Peak Freq: %d.%03d Hz\n", (int)(d.freq), (int)(d.freq * 1000) % 1000);
    // printf("Peak Mag : %d.%03d\n", (int)(d.magnitude), (int)(d.magnitude * 1000) % 1000);

    if (d.detected) {
        printf("[Tremor Detected]\n");
    } else {
        printf("[No Tremor]\n");
    }

    return d;
}


// 舞蹈症检测：同上 
Detection detectDyskinesiaFFT(const float32_t gyro_buf[]) {
    // Step 1: 去除 DC 分量
    float32_t gyro_buf_dcd[FFT_SIZE];
    float32_t mean = 0.0f;
    
    // 计算均值
    for (int i = 0; i < FFT_SIZE; i++) {
        mean += gyro_buf[i];
    }
    mean /= FFT_SIZE;

    // 去除 DC 分量
    for (int i = 0; i < FFT_SIZE; i++) {
        gyro_buf_dcd[i] = gyro_buf[i] - mean;
    }

    // Step 2: 打印去除 DC 分量后的 gyro_buf（可选）
    // for (int i = 0; i < 10; i++) {
    //     printf("gyro_buf[%d] = %d.%03d\n", i, (int)(gyro_buf_dcd[i]), (int)(gyro_buf_dcd[i] * 1000) % 1000);
    // }

    // Step 3: 做 FFT
    computeFFT(gyro_buf_dcd, mag_gyro);

    // Step 4: 打印 FFT 结果
    // for (int i = 0; i < FFT_SIZE / 2; i++) {
    //     printf("FFT Bin %d: %f\n", i, mag_gyro[i]);
    // }

    // Step 5: 找峰值
    uint32_t idx; 
    float32_t val;
    arm_max_f32(mag_gyro, FFT_SIZE / 2, &val, &idx);

    // Step 6: 填结果
    Detection d;
    d.freq      = idx * (SAMPLE_RATE / FFT_SIZE);
    d.magnitude = val;

    // Step 7: 判断频段范围 0.5 ~ 3 Hz（异动症）
    const float32_t DYSK_FREQ_MIN = 1.0f;
    const float32_t DYSK_FREQ_MAX = 3.0f;
    const float32_t DYSK_MAG_THRESHOLD = 10000.0f;
    d.detected = (
        d.freq > DYSK_FREQ_MIN && 
        d.freq <= DYSK_FREQ_MAX &&
        d.magnitude >= DYSK_MAG_THRESHOLD
    ); // 允许的异动症频段范围

    if (d.detected) {
        printf("[Dyskinesia Detected]\n");
    } else {
        printf("[No Dyskinesia]\n");
    }

    return d;
}
int fft_index = 0;
void sample_and_process() {
    // 读取传感器原始值
    int16_t acc_x_raw = read_16bit_value(OUTX_L_XL, OUTX_H_XL);
    int16_t acc_y_raw = read_16bit_value(OUTY_L_XL, OUTY_H_XL);
    int16_t acc_z_raw = read_16bit_value(OUTZ_L_XL, OUTZ_H_XL);

    int16_t gyro_x_raw = read_16bit_value(OUTX_L_G, OUTX_H_G);
    int16_t gyro_y_raw = read_16bit_value(OUTY_L_G, OUTY_H_G);
    int16_t gyro_z_raw = read_16bit_value(OUTZ_L_G, OUTZ_H_G);

    // 单位转换
    float acc_x_g = transformAccelerometer(acc_x_raw);
    float acc_y_g = transformAccelerometer(acc_y_raw);
    float acc_z_g = transformAccelerometer(acc_z_raw);

    float gyro_x_dps = tranformGyroscope(gyro_x_raw);
    float gyro_y_dps = tranformGyroscope(gyro_y_raw);
    float gyro_z_dps = tranformGyroscope(gyro_z_raw);

    // 计算模长
    float acc_mag = sqrt(acc_x_g * acc_x_g + acc_y_g * acc_y_g + acc_z_g * acc_z_g);
    float gyro_mag = sqrt(gyro_x_dps * gyro_x_dps + gyro_y_dps * gyro_y_dps + gyro_z_dps * gyro_z_dps);

    // 存入 FFT 输入数组
    input_acc[fft_index]  = (int)(acc_mag * 1000);
    input_gyro[fft_index] = (int)(gyro_mag * 1000);
    fft_index++;

    // 到达 FFT 数据窗口后进行处理
    if (fft_index >= FFT_SIZE) {
        Detection tremor = detectTremorFFT(input_acc);
        Detection dysk   = detectDyskinesiaFFT(input_gyro);
        fft_index = 0;


        // 本地串口打印信息
        if (tremor.detected) {
            int freq_int = (int)(tremor.freq * 100);         // 保留两位小数
            int mag_int = (int)(tremor.magnitude * 10000);   // 保留四位小数
            printf("[Tremor Detected] Peak Frequency: %d (x0.01 Hz) | Magnitude: %d (x0.0001)\r\n", freq_int, mag_int);
        }

        if (dysk.detected) {
            int freq_int = (int)(dysk.freq * 100);
            int mag_int = (int)(dysk.magnitude * 10000);
            printf("[Dyskinesia Detected] Peak Frequency: %d (x0.01 Hz) | Magnitude: %d (x0.0001)\r\n", freq_int, mag_int);
        }

        // BLE 通知连接设备
        if (!is_connected) {
            printf("[ERROR] Not connected to BLE device!\r\n");
            return;
        }
        if (is_connected) {
            if (!GattCharacteristic::isWritable(tremorChar.getProperties())) {
    printf("[ERROR] GATT characteristic is not writable.\r\n");
} 
    uint8_t msg[2]; // 使用 2 字节，分别表示 Tremor 和 Dyskinesia 的状态

    // 打印连接状态
    printf("[DEBUG] is_connected = %d\n", is_connected);

    // 打印 tremor 和 dysk 的检测状态
    printf("[DEBUG] tremor.detected = %d, dysk.detected = %d\n", tremor.detected, dysk.detected);

    // 设置 tremor 和 dysk 的数值
    msg[0] = tremor.detected ? 1 : 0;  // 1 表示震颤检测，0 表示无震颤
    msg[1] = dysk.detected ? 2 : 0;    // 2 表示异动症检测，0 表示无异动症

    // 打印 msg 数组的内容
    printf("[DEBUG] msg[0] = %d, msg[1] = %d\n", msg[0], msg[1]);

    // 打印 GATT 特征句柄
    printf("[DEBUG] GATT Characteristic Handle: %d\n", tremorChar.getValueHandle());

    // 写入数据
    ble_error_t err = ble_interface.gattServer().write(
        tremorChar.getValueHandle(),
        msg,  // 发送原始的数值
        sizeof(msg)  // 发送 2 字节数据
    );

    
    // 检查 BLE 写入错误
    if (err) {
        printf("[ERROR] BLE write failed: %d\r\n", err);
    } else {
        printf("[BLE Notify] Tremor: %d Dyskinesia: %d\r\n", msg[0], msg[1]);
    }
} else {
    printf("[ERROR] Not connected to BLE device!\r\n");
}


    }
}



int main(){
    ThisThread::sleep_for(500ms);  // 给串口初始化留时间
    printf("Starting system...\r\n");
    printf("Starting BLE Tremor Monitor...\n");


    // 初始化 BLE
    ble_interface.onEventsToProcess(schedule_ble_events);
    ble_interface.init(on_init_complete);


    //初始化硬件
    init_I2C();

    // 检查传感器是否连接
    checkconnection();

    // 配置传感器
    configure_sensor();
    //初始化 ARM FFT 实例
    arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE);
    constexpr auto SAMPLE_PERIOD = 1000ms / 104;
    event_queue.call_every(SAMPLE_PERIOD, sample_and_process);
    event_queue.dispatch_forever();
}
