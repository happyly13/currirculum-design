#include "bluetooth.h"
#include "usart.h"
#include "dma.h"
#include "sw_clock.h"
#include <string.h>
#include <stdio.h>

extern DMA_HandleTypeDef hdma_usart2_rx;

/* Ring buffer for DMA RX */
static uint8_t rx_buf[BT_RX_BUF_SIZE];
static uint8_t tx_buf[BT_TX_BUF_SIZE];
static uint16_t rx_old_pos = 0;

/* Frame parser state */
static uint8_t frame_buf[BT_RX_BUF_SIZE];
static uint8_t frame_idx = 0;
static uint8_t frame_expected_len = 0;

/* Received time sync data */
static BT_TimeSync_t rx_time_sync;
static volatile uint8_t time_sync_available = 0;

static void BT_ParseByte(uint8_t byte);
static void BT_ParseTextCmd(uint8_t byte);

void BT_Init(void)
{
    /* Start DMA circular reception */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, BT_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

/* UART RX event callback (IDLE line or half-transfer) */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2 && Size > 0)
    {
        BT_RxCpltCallback(Size);
    }
}

void BT_RxCpltCallback(uint16_t size)
{
    uint16_t new_pos = size;

    if (new_pos > BT_RX_BUF_SIZE)
    {
        new_pos = BT_RX_BUF_SIZE;
    }

    if (new_pos == rx_old_pos)
    {
        return;
    }

    if (new_pos > rx_old_pos)
    {
        for (uint16_t i = rx_old_pos; i < new_pos; i++)
        {
            BT_ParseByte(rx_buf[i]);
            BT_ParseTextCmd(rx_buf[i]);
        }
    }
    else
    {
        for (uint16_t i = rx_old_pos; i < BT_RX_BUF_SIZE; i++)
        {
            BT_ParseByte(rx_buf[i]);
            BT_ParseTextCmd(rx_buf[i]);
        }
        for (uint16_t i = 0; i < new_pos; i++)
        {
            BT_ParseByte(rx_buf[i]);
            BT_ParseTextCmd(rx_buf[i]);
        }
    }

    rx_old_pos = new_pos;
}

/* ---- Text command parser ("T14:30:00") ---- */
static char   cmd_buf[16];
static uint8_t cmd_idx = 0;

static void BT_ParseTextCmd(uint8_t byte)
{
    /* Line ending → process command */
    if (byte == '\r' || byte == '\n') {
        if (cmd_idx == 0) return;
        cmd_buf[cmd_idx] = '\0';
    }
    /* "T14:30:00" has exactly 9 chars → process on 9th byte */
    else if (cmd_idx == 8) {
        cmd_buf[8] = (char)byte;
        cmd_buf[9] = '\0';
        cmd_idx = 9;
    }
    /* Overflow or garbage → reset */
    else if (cmd_idx >= sizeof(cmd_buf) - 1) {
        cmd_idx = 0;
        return;
    }
    /* Normal char → buffer it */
    else {
        cmd_buf[cmd_idx++] = (char)byte;
        return;
    }

    /* Echo command back for debug */
    cmd_buf[cmd_idx] = '\0';
    HAL_UART_Transmit(&huart2, (uint8_t*)"[", 1, 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd_buf, cmd_idx, 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"]\r\n", 3, 100);

    /* Try to parse time sync: "T14:30:00" */
    if (cmd_buf[0] == 'T' && cmd_idx >= 9) {
        uint8_t h = (uint8_t)((cmd_buf[1]-'0')*10 + (cmd_buf[2]-'0'));
        uint8_t m = (uint8_t)((cmd_buf[4]-'0')*10 + (cmd_buf[5]-'0'));
        uint8_t s = (uint8_t)((cmd_buf[7]-'0')*10 + (cmd_buf[8]-'0'));
        if (h < 24 && m < 60 && s < 60) {
            SWClock_Sync(h, m, s);
        }
    }

    cmd_idx = 0;
}

static void BT_ParseByte(uint8_t byte)
{
    if (frame_idx == 0)
    {
        if (byte == BT_STX)
        {
            frame_buf[frame_idx++] = byte;
        }
        return;
    }

    if (frame_idx == 1)
    {
        frame_buf[frame_idx++] = byte; /* CMD */
        return;
    }

    if (frame_idx == 2)
    {
        frame_expected_len = byte;
        if (frame_expected_len > (BT_RX_BUF_SIZE - 5))
        {
            frame_idx = 0;
            frame_expected_len = 0;
            return;
        }
        frame_buf[frame_idx++] = byte; /* LEN */
        return;
    }

    if (frame_idx < 3 + frame_expected_len)
    {
        frame_buf[frame_idx++] = byte; /* DATA */
        return;
    }

    if (frame_idx == 3 + frame_expected_len)
    {
        frame_buf[frame_idx++] = byte; /* CHK */
        return;
    }

    if (frame_idx == 4 + frame_expected_len)
    {
        frame_buf[frame_idx] = byte; /* ETX */
        if (byte == BT_ETX)
        {
            uint8_t chk = frame_buf[1] ^ frame_buf[2];
            for (uint8_t j = 0; j < frame_expected_len; j++)
            {
                chk ^= frame_buf[3 + j];
            }
            if (chk == frame_buf[3 + frame_expected_len])
            {
                uint8_t cmd = frame_buf[1];
                if (cmd == BT_CMD_TIME_SYNC && frame_expected_len == 7)
                {
                    rx_time_sync.hour    = frame_buf[3];
                    rx_time_sync.minute  = frame_buf[4];
                    rx_time_sync.second  = frame_buf[5];
                    rx_time_sync.year    = ((uint16_t)frame_buf[6] << 8) | frame_buf[7];
                    rx_time_sync.month   = frame_buf[8];
                    rx_time_sync.day     = frame_buf[9];
                    rx_time_sync.weekday = 0;
                    time_sync_available = 1;
                }
            }
        }
        frame_idx = 0;
        frame_expected_len = 0;
    }
}

/* Send sensor data frame via BT */
void BT_SendSensorData(SmartWatchData_t *data)
{
    /* Build frame: STX + CMD + LEN + DATA + CHK + ETX */
    uint8_t len = 6 * 4; /* 6 floats = 24 bytes */
    tx_buf[0] = BT_STX;
    tx_buf[1] = BT_CMD_SENSOR_DATA;
    tx_buf[2] = len;

    /* Pack sensor data (little-endian float) */
    uint8_t *p = &tx_buf[3];
    memcpy(p, &data->accel.ax, 4); p += 4;
    memcpy(p, &data->accel.ay, 4); p += 4;
    memcpy(p, &data->accel.az, 4); p += 4;
    memcpy(p, &data->gyro.gx,  4); p += 4;
    memcpy(p, &data->gyro.gy,  4); p += 4;
    memcpy(p, &data->gyro.gz,  4); /* p += 4; */

    /* Checksum */
    uint8_t chk = tx_buf[1] ^ tx_buf[2];
    for (uint8_t i = 0; i < len; i++)
    {
        chk ^= tx_buf[3 + i];
    }
    tx_buf[3 + len] = chk;
    tx_buf[4 + len] = BT_ETX;

    /* Send via DMA (non-blocking) */
    HAL_UART_Transmit_DMA(&huart2, tx_buf, 5 + len);
}

/* Check and apply time sync from phone */
uint8_t BT_GetTimeSync(BT_TimeSync_t *ts)
{
    if (time_sync_available)
    {
        memcpy(ts, &rx_time_sync, sizeof(BT_TimeSync_t));
        time_sync_available = 0;
        return 1;
    }
    return 0;
}

uint8_t BT_Process(SmartWatchData_t *data)
{
    if (data == NULL) return 0;

    /* New time sync frame → update sw_clock base + date */
    BT_TimeSync_t ts;
    if (BT_GetTimeSync(&ts))
    {
        SWClock_Sync(ts.hour, ts.minute, ts.second);
        data->year    = ts.year;
        data->month   = ts.month;
        data->day     = ts.day;
        data->weekday = ts.weekday;
    }

    /* Feed current time to data (no-op if not synced) */
    if (SWClock_IsSynced())
    {
        SWClock_GetTime(&data->hour, &data->minute, &data->second);
        return 1;
    }

    return 0;
}

/* Read HC-05 STATE pin: returns 1 if connected, 0 if disconnected */
uint8_t BT_IsConnected(void)
{
    return (HAL_GPIO_ReadPin(BT_STATE_PORT, BT_STATE_PIN) == GPIO_PIN_SET) ? 1 : 0;
}
