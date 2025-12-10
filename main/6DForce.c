/**
 * @file rs485_example.c
 * @brief RS485传感器数据读取程序
 * 
 * 本程序通过RS485接口读取传感器数据（被动读取模式）
 * - ESP32作为上位机，主动发送读取指令
 * - 传感器响应后返回29字节数据
 * - 解析6个维度的力/力矩数据（Fx, Fy, Fz, Mx, My, Mz）
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/twai.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define TAG "RS485_SENSOR"

/** @brief 传感器左右手定义 */
#define SENSOR_HAND_RIGHT     (0x71)  /**< 右手传感器ID */
#define SENSOR_HAND_LEFT      (0x72)  /**< 左手传感器ID */

/** @brief 当前使用的传感器手型（可通过修改此宏切换左右手） */
#define SENSOR_HAND           (SENSOR_HAND_LEFT)  /**< 默认使用右手传感器 */

/** @brief CAN引脚定义 */
#define CAN_TX_PIN            (6)     /**< CAN发送引脚 */
#define CAN_RX_PIN            (7)     /**< CAN接收引脚 */
#define CAN_ID                (SENSOR_HAND)  /**< CAN帧ID（根据左右手自动设置） */

/** @brief CAN帧头定义 */
#define CAN_FRAME_HEADER_F    (0x01)  /**< 力数据帧头 */
#define CAN_FRAME_HEADER_M    (0x02)  /**< 力矩数据帧头 */

/** @brief UART引脚定义 */
#define UART_TXD            (9)     /**< 发送引脚 */
#define UART_RXD            (8)     /**< 接收引脚 */
#define UART_RTS            (10)     /**< RTS引脚，用于RS485收发控制 */
#define UART_CTS            (-1)     /**< CTS引脚，RS485半双工模式不使用 */

/** @brief UART配置参数 */
#define UART_PORT_NUM       (1)     /**< UART端口号 */
#define BUF_SIZE            (127)   /**< 接收缓冲区大小 */
#define BAUD_RATE           (115200)/**< 波特率 */

/** @brief 任务配置 */
#define TASK_STACK_SIZE     (8192)  /**< 任务栈大小 */
#define TASK_PRIO           (10)    /**< 任务优先级 */

/** @brief 超时配置 */
#define READ_TIMEOUT_MS     (100)   /**< 读取超时时间（毫秒） */
#define READ_TIMEOUT_TICKS  (READ_TIMEOUT_MS / portTICK_PERIOD_MS)
#define UART_READ_TOUT      (3)     /**< UART接收超时阈值 */

/** @brief 传感器通信协议定义 */
#define SENSOR_STATION_ID   (0x01)  /**< 传感器站号（默认01，站号为9时改为0x09） */
#define MODBUS_FUNC_READ    (0x04)  /**< Modbus功能码：读取模拟量 */
#define READ_START_ADDR     (0x0000)/**< 起始地址 */
#define READ_REG_COUNT      (0x000C)/**< 读取寄存器数量（12个，对应6个浮点数） */
#define READ_CMD_LEN        (8)     /**< 读取指令长度 */
#define RESPONSE_LEN        (29)    /**< 传感器响应数据长度 */
#define DATA_LEN_BYTE       (0x18)  /**< 数据长度字节（固定24字节，即0x18） */

/** @brief 读取指令：01 04 00 00 00 0C F0 0F */
static const uint8_t read_cmd[READ_CMD_LEN] = {
    0x01, 0x04, 0x00, 0x00, 0x00, 0x0C, 0xF0, 0x0F
};

/** @brief 传感器数据结构 */
typedef struct {
    float fx;   /**< X轴力（N） */
    float fy;   /**< Y轴力（N） */
    float fz;   /**< Z轴力（N） */
    float mx;   /**< X轴力矩（N·m） */
    float my;   /**< Y轴力矩（N·m） */
    float mz;   /**< Z轴力矩（N·m） */
} sensor_data_t;

/**
 * @brief 计算CRC16_MODBUS校验码
 * 
 * @param data 待校验的数据
 * @param length 数据长度
 * @return uint16_t CRC16校验码
 */
static uint16_t crc16_modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief 发送数据到UART
 * 
 * @param port UART端口号
 * @param data 待发送数据
 * @param length 数据长度
 * @return esp_err_t 错误码
 */
static esp_err_t uart_send_data(int port, const uint8_t *data, size_t length)
{
    int sent = uart_write_bytes(port, data, length);
    if (sent != length) {
        ESP_LOGE(TAG, "发送数据失败，期望发送 %d 字节，实际发送 %d 字节", length, sent);
        return ESP_FAIL;
    }
    
    // 等待发送完成
    esp_err_t ret = uart_wait_tx_done(port, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "等待发送完成失败");
        return ret;
    }
    
    return ESP_OK;
}

/**
 * @brief 发送读取指令到传感器
 * 
 * @param port UART端口号
 * @return esp_err_t 错误码
 */
static esp_err_t send_read_command(int port)
{
    ESP_LOGD(TAG, "发送读取指令: ");
    for (int i = 0; i < READ_CMD_LEN; i++) {
        ESP_LOGD(TAG, "0x%02X ", read_cmd[i]);
    }
    ESP_LOGD(TAG, "");
    
    return uart_send_data(port, read_cmd, READ_CMD_LEN);
}

/**
 * @brief 将大端序的4字节数据转换为float（Modbus协议使用大端序）
 * 
 * @param bytes 4字节数据（大端序）
 * @return float 转换后的浮点数
 */
static float bytes_to_float_big_endian(const uint8_t *bytes)
{
    union {
        uint8_t bytes[4];
        float value;
    } converter;
    
    // 将大端序字节转换为小端序（ESP32是小端序）
    converter.bytes[0] = bytes[3];
    converter.bytes[1] = bytes[2];
    converter.bytes[2] = bytes[1];
    converter.bytes[3] = bytes[0];
    
    return converter.value;
}

/**
 * @brief 解析传感器返回的29字节数据
 * 
 * @param data 接收到的数据（29字节）
 * @param sensor_data 解析后的传感器数据结构指针
 * @return esp_err_t 错误码
 */
static esp_err_t parse_sensor_data(const uint8_t *data, sensor_data_t *sensor_data)
{
    // 检查数据长度
    if (data[2] != DATA_LEN_BYTE) {
        ESP_LOGE(TAG, "数据长度错误，期望 0x%02X，实际 0x%02X", DATA_LEN_BYTE, data[2]);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // 检查站号和功能码
    if (data[0] != SENSOR_STATION_ID) {
        ESP_LOGW(TAG, "站号不匹配，期望 0x%02X，实际 0x%02X", SENSOR_STATION_ID, data[0]);
    }
    
    if (data[1] != MODBUS_FUNC_READ) {
        ESP_LOGE(TAG, "功能码错误，期望 0x%02X，实际 0x%02X", MODBUS_FUNC_READ, data[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // 解析6个浮点数（每4字节一个，高字节在前，MSB在前，即大端序）
    // Modbus协议使用大端序，需要转换为小端序（ESP32是小端序）
    // Fx: 字节 3-6
    sensor_data->fx = bytes_to_float_big_endian(&data[3]);
    
    // Fy: 字节 7-10
    sensor_data->fy = bytes_to_float_big_endian(&data[7]);
    
    // Fz: 字节 11-14
    sensor_data->fz = bytes_to_float_big_endian(&data[11]);
    
    // Mx: 字节 15-18
    sensor_data->mx = bytes_to_float_big_endian(&data[15]);
    
    // My: 字节 19-22
    sensor_data->my = bytes_to_float_big_endian(&data[19]);
    
    // Mz: 字节 23-26
    sensor_data->mz = bytes_to_float_big_endian(&data[23]);
    
    // 注意：最后2字节（27-28）是CRC校验码，这里不进行校验
    
    return ESP_OK;
}

/**
 * @brief 打印传感器数据
 * 
 * @param sensor_data 传感器数据结构指针
 */
static void print_sensor_data(const sensor_data_t *sensor_data)
{
    ESP_LOGI(TAG, "========== 传感器数据 ==========");
    ESP_LOGI(TAG, "Fx: %.5f N", sensor_data->fx);
    ESP_LOGI(TAG, "Fy: %.5f N", sensor_data->fy);
    ESP_LOGI(TAG, "Fz: %.5f N", sensor_data->fz);
    ESP_LOGI(TAG, "Mx: %.5f N·m", sensor_data->mx);
    ESP_LOGI(TAG, "My: %.5f N·m", sensor_data->my);
    ESP_LOGI(TAG, "Mz: %.5f N·m", sensor_data->mz);
    ESP_LOGI(TAG, "================================");
}

/**
 * @brief 将float力值转换为int16（乘以100去除小数）
 * 
 * @param force float类型的力值（N）
 * @return int16_t 转换后的int16值（实际值 = force * 100）
 */
static int16_t force_to_int16(float force)
{
    // 将力值乘以100，然后转换为int16
    float scaled = force * 100.0f;
    
    // 限制范围在int16范围内（-32768到32767）
    if (scaled > 32767.0f) {
        return 32767;
    } else if (scaled < -32768.0f) {
        return -32768;
    }
    return (int16_t)scaled;
}

/**
 * @brief 初始化CAN驱动
 * 
 * @return esp_err_t 错误码
 */
static esp_err_t can_init(void)
{
    /** @brief TWAI配置结构体 */
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    /** @brief TWAI时序配置结构体（1Mbps） */
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    /** @brief TWAI滤波配置结构体 */
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // 安装TWAI驱动
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN驱动安装失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 启动TWAI驱动
    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN驱动启动失败: %s", esp_err_to_name(ret));
        twai_driver_uninstall();
        return ret;
    }
    
    // 等待CAN总线稳定
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "CAN驱动初始化成功");
    return ESP_OK;
}

/**
 * @brief 发送传感器力数据（F）到CAN总线
 * 
 * @param sensor_data 传感器数据结构指针
 * @return esp_err_t 错误码
 */
static esp_err_t send_force_data_to_can(const sensor_data_t *sensor_data)
{
    /** @brief CAN消息结构体 */
    twai_message_t message;
    message.identifier = CAN_ID;
    message.flags = TWAI_MSG_FLAG_NONE;
    message.data_length_code = 7;  // 帧头1字节 + 3个int16 = 7字节
    
    // 将3个方向的力转换为int16并填充到CAN数据中
    int16_t fx_int16 = force_to_int16(sensor_data->fx);
    int16_t fy_int16 = force_to_int16(sensor_data->fy);
    int16_t fz_int16 = force_to_int16(sensor_data->fz);
    
    // 填充CAN数据：帧头 + 3个int16（小端序）
    message.data[0] = CAN_FRAME_HEADER_F;  // 帧头：力数据
    message.data[1] = (uint8_t)(fx_int16 & 0xFF);
    message.data[2] = (uint8_t)((fx_int16 >> 8) & 0xFF);
    message.data[3] = (uint8_t)(fy_int16 & 0xFF);
    message.data[4] = (uint8_t)((fy_int16 >> 8) & 0xFF);
    message.data[5] = (uint8_t)(fz_int16 & 0xFF);
    message.data[6] = (uint8_t)((fz_int16 >> 8) & 0xFF);
    
    // 检查CAN总线状态
    twai_status_info_t status_info;
    twai_get_status_info(&status_info);
    if (status_info.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGE(TAG, "CAN总线处于BUS_OFF状态，尝试恢复");
        twai_initiate_recovery();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // 发送CAN消息（增加超时时间到500ms）
    esp_err_t ret = twai_transmit(&message, pdMS_TO_TICKS(500));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN发送力数据失败: %s (状态: %d, 错误: 0x%08X)", 
                 esp_err_to_name(ret), status_info.state, (unsigned int)status_info.bus_error_count);
        return ret;
    }
    
    ESP_LOGI(TAG, "CAN力数据帧已发送: ID=0x%02X, Header=0x%02X, Fx=%d, Fy=%d, Fz=%d", 
             CAN_ID, CAN_FRAME_HEADER_F, fx_int16, fy_int16, fz_int16);
    
    return ESP_OK;
}

/**
 * @brief 发送传感器力矩数据（M）到CAN总线
 * 
 * @param sensor_data 传感器数据结构指针
 * @return esp_err_t 错误码
 */
static esp_err_t send_moment_data_to_can(const sensor_data_t *sensor_data)
{
    /** @brief CAN消息结构体 */
    twai_message_t message;
    message.identifier = CAN_ID;
    message.flags = TWAI_MSG_FLAG_NONE;
    message.data_length_code = 7;  // 帧头1字节 + 3个int16 = 7字节
    
    // 将3个方向的力矩转换为int16并填充到CAN数据中
    int16_t mx_int16 = force_to_int16(sensor_data->mx);
    int16_t my_int16 = force_to_int16(sensor_data->my);
    int16_t mz_int16 = force_to_int16(sensor_data->mz);
    
    // 填充CAN数据：帧头 + 3个int16（小端序）
    message.data[0] = CAN_FRAME_HEADER_M;  // 帧头：力矩数据
    message.data[1] = (uint8_t)(mx_int16 & 0xFF);
    message.data[2] = (uint8_t)((mx_int16 >> 8) & 0xFF);
    message.data[3] = (uint8_t)(my_int16 & 0xFF);
    message.data[4] = (uint8_t)((my_int16 >> 8) & 0xFF);
    message.data[5] = (uint8_t)(mz_int16 & 0xFF);
    message.data[6] = (uint8_t)((mz_int16 >> 8) & 0xFF);
    
    // 检查CAN总线状态
    twai_status_info_t status_info;
    twai_get_status_info(&status_info);
    if (status_info.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGE(TAG, "CAN总线处于BUS_OFF状态，尝试恢复");
        twai_initiate_recovery();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // 发送CAN消息（增加超时时间到500ms）
    esp_err_t ret = twai_transmit(&message, pdMS_TO_TICKS(500));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN发送力矩数据失败: %s (状态: %d, 错误: 0x%08X)", 
                 esp_err_to_name(ret), status_info.state, (unsigned int)status_info.bus_error_count);
        return ret;
    }
    
    ESP_LOGI(TAG, "CAN力矩数据帧已发送: ID=0x%02X, Header=0x%02X, Mx=%d, My=%d, Mz=%d", 
             CAN_ID, CAN_FRAME_HEADER_M, mx_int16, my_int16, mz_int16);
    
    return ESP_OK;
}

/**
 * @brief 打印接收到的原始数据（十六进制格式）
 * 
 * @param data 数据指针
 * @param length 数据长度
 */
static void print_raw_data(const uint8_t *data, size_t length)
{
    ESP_LOGI(TAG, "接收到 %d 字节数据:", length);
    printf("[ ");
    for (size_t i = 0; i < length; i++) {
        printf("0x%02X ", data[i]);
    }
    printf("]\n");
}

/**
 * @brief RS485传感器读取任务
 * 
 * @param arg 任务参数（未使用）
 */
static void sensor_read_task(void *arg)
{
    const int uart_num = UART_PORT_NUM;
    
    /** @brief UART配置结构体 */
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // 设置日志级别
    esp_log_level_set(TAG, ESP_LOG_INFO);
    
    ESP_LOGI(TAG, "初始化RS485传感器读取程序");
    
    // 安装UART驱动
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));
    
    // 配置UART参数
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    
    // 设置UART引脚
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_TXD, UART_RXD, UART_RTS, UART_CTS));
    
    // 设置RS485半双工模式
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));
    
    // 设置接收超时
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, UART_READ_TOUT));
    
    ESP_LOGI(TAG, "UART配置完成，开始读取传感器数据");
    
    // 分配接收缓冲区
    uint8_t *rx_buffer = (uint8_t*)malloc(BUF_SIZE);
    if (rx_buffer == NULL) {
        ESP_LOGE(TAG, "分配接收缓冲区失败");
        vTaskDelete(NULL);
        return;
    }
    
    sensor_data_t sensor_data = {0};
    TickType_t last_send_time = 0;
    const TickType_t send_interval = pdMS_TO_TICKS(30); // 每30ms发送一次读取指令
    
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        // 检查是否需要发送读取指令（每100ms发送一次）
        if ((current_time - last_send_time) >= send_interval) {
            // 清空接收缓冲区
            uart_flush(uart_num);
            
            // 发送读取指令
            if (send_read_command(uart_num) == ESP_OK) {
                last_send_time = current_time;
                ESP_LOGI(TAG, "已发送读取指令，等待传感器响应...");
            } else {
                ESP_LOGE(TAG, "发送读取指令失败");
            }
        }
        
        // 尝试接收数据
        int len = uart_read_bytes(uart_num, rx_buffer, BUF_SIZE, READ_TIMEOUT_TICKS);
        
        if (len > 0) {
            // 打印原始数据
            //print_raw_data(rx_buffer, len);
            
            // 检查数据长度
            if (len == RESPONSE_LEN) {
                // 解析传感器数据
                if (parse_sensor_data(rx_buffer, &sensor_data) == ESP_OK) {
                    // 打印解析后的数据
                    print_sensor_data(&sensor_data);
                    
                    // 发送力数据（F）到CAN总线
                    if (send_force_data_to_can(&sensor_data) != ESP_OK) {
                        ESP_LOGW(TAG, "CAN发送力数据失败，但继续运行");
                    }
                    
                    // 发送力矩数据（M）到CAN总线
                    if (send_moment_data_to_can(&sensor_data) != ESP_OK) {
                        ESP_LOGW(TAG, "CAN发送力矩数据失败，但继续运行");
                    }
                } else {
                    ESP_LOGE(TAG, "解析传感器数据失败");
                }
            } else {
                ESP_LOGW(TAG, "接收数据长度不正确，期望 %d 字节，实际 %d 字节", RESPONSE_LEN, len);
            }
        }
        
        // 短暂延时，避免CPU占用过高
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // 释放资源（理论上不会执行到这里）
    free(rx_buffer);
    vTaskDelete(NULL);
}

/**
 * @brief 应用程序入口函数
 */
void app_main(void)
{
    ESP_LOGI(TAG, "RS485传感器读取程序启动");
    
    // 初始化CAN驱动
    if (can_init() != ESP_OK) {
        ESP_LOGE(TAG, "CAN初始化失败，程序将继续运行但无法发送CAN数据");
    }
    
    // 创建传感器读取任务
    xTaskCreatePinnedToCore(sensor_read_task, "sensor_read_task", TASK_STACK_SIZE, NULL, TASK_PRIO, NULL, 0);
    
    ESP_LOGI(TAG, "传感器读取任务已创建");
}
