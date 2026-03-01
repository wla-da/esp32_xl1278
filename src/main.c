#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "SX1278";

/* ===================== GPIO CONFIG ===================== */

// SPI pins (VSPI)
#define PIN_MISO   19
#define PIN_MOSI   23
#define PIN_SCK    18
#define PIN_NSS    5

// SX1278 control
#define PIN_RST    14
#define PIN_DIO0   27
#define PIN_DIO1   26
#define PIN_DIO2   25

/* ===================== SX1278 REGISTERS ===================== */

// Common
#define REG_FIFO                    0x00
#define REG_VERSION                 0x42

#define REG_OP_MODE                 0x01
// Основные режимы
#define MODE_SLEEP                  0x00
#define MODE_STDBY                  0x01
#define MODE_TX                     0x03


// FSK / GMSK
#define REG_BITRATE_MSB             0x02
#define REG_BITRATE_LSB             0x03
#define REG_FDEV_MSB                0x04
#define REG_FDEV_LSB                0x05
#define REG_FRF_MSB                 0x06
#define REG_FRF_MID                 0x07
#define REG_FRF_LSB                 0x08
#define REG_PA_CONFIG               0x09
#define SX1278_PA_SELECT_BOOST      0x80   // bit7 = 1 → PA_BOOST
#define SX1278_MAX_POWER            0x70   // bits6:4 = 111 (максимум)
#define REG_PA_RAMP                 0x0A
#define SX1278_PA_RAMP_3_4MS        0x00
#define SX1278_PA_RAMP_2MS          0x01
#define SX1278_PA_RAMP_1MS          0x02 // PA ramp: 1 ms (AIS friendly)
#define SX1278_PA_RAMP_500US        0x03
#define SX1278_PA_RAMP_250US        0x04
#define SX1278_PA_RAMP_125US        0x05
#define SX1278_PA_RAMP_100US        0x06
#define SX1278_PA_RAMP_62US         0x07
#define SX1278_MOD_SHAPING_NONE     (0x0 << 5)  // 0x00
#define SX1278_GMSK_BT_1_0          (0x1 << 5)  // 0x20
#define SX1278_GMSK_BT_0_5          (0x2 << 5)  // 0x40
#define SX1278_GMSK_BT_0_3          (0x3 << 5)  // 0x60 Modulation shaping: Gaussian BT = 0.3 (AIS compliant)
#define REG_LNA                     0x0C
#define REG_RX_CONFIG               0x0D
#define REG_RSSI_CONFIG             0x0E
#define REG_PACKET_CONFIG_1         0x30
#define REG_PACKET_CONFIG_2         0x31
#define PACKET_MODE_ENABLE          (1 << 6)
#define REG_PAYLOAD_LENGTH          0x32
#define REG_FIFO_THRESH             0x35
#define FIFO_TX_START_COND_FULL     (1 << 7)
#define REG_PACKET_MODE             0x37
#define REG_IRQ_FLAGS_1             0x3E
#define REG_IRQ_FLAGS_2             0x3F
#define SX1278_IRQ2_PACKET_SENT     (1U << 3)  //битовая маска для бита 3: 1 - пакет отправлен, 0 - идет передача


#define AIS_BITRATE                 9600        //битрейт для AIS в бит/сек
#define AIS_FDEV_HZ                 2400        //девиация частоты AIS ±2.4 кГц (h*AIS_BITRATE/2, где h=0.5 индекс модуляции для GMSK)
#define RF_FREQUENCY_HZ             162000000UL //частота настройки передатчика 162 МГц ровно
#define RF_POWER_DBM                2 //выходная мощность в дБм в диапазоне +2..+17
#define XTAL_FREQ_HZ                32000000UL
#define  FRF_STEP_HZ                 (XTAL_FREQ_HZ / (1UL << 19))  // ≈ 61.03515625 Гц

#define MAX_PACKET_LEN              24

/* ===================== SPI ===================== */

static spi_device_handle_t spi;

/* ===================== LOW LEVEL SPI ===================== */

static uint8_t sx1278_read(uint8_t addr)
{
    uint8_t tx[2] = { addr & 0x7F, 0x00 };
    uint8_t rx[2] = { 0 };

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    spi_device_transmit(spi, &t);
    return rx[1];
}

static void sx1278_write(uint8_t addr, uint8_t value)
{
    uint8_t tx[2] = { addr | 0x80, value };

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx
    };

    spi_device_transmit(spi, &t);
}

/* ===================== RESET ===================== */

static void sx1278_reset(void)
{
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}



/* ============================================================
 * CRC-16/X25 (AIS / HDLC / ISO 3309)
 *
 * Полином (по стандарту): 0x1021
 * Реализация: LSB-first (reflected)
 *
 * Параметры:
 *   Init   = 0xFFFF
 *   RefIn  = true
 *   RefOut = true
 *   XorOut = 0xFFFF
 *
 * Примечание:
 *   В reflected-реализации используется
 *   битово-отражённый полином 0x8408,
 *   что эквивалентно 0x1021.
 * ============================================================
 */

static uint16_t ais_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++) {

        /* XOR с младшим байтом CRC (LSB-first) */
        crc ^= data[i];

        for (int bit = 0; bit < 8; bit++) {

            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0x8408;  // отражённый 0x1021
            } else {
                crc >>= 1;
            }
        }
    }

    /* Финальная инверсия согласно X25 */
    crc ^= 0xFFFF;

    return crc;
}




/* ===================== BIT STUFFING ===================== */

static size_t bit_stuff(const uint8_t *in, size_t in_bits, uint8_t *out)
{
    size_t out_bit = 0;
    int ones = 0;

    for (size_t i = 0; i < in_bits; i++) {
        bool bit = (in[i / 8] >> (7 - (i % 8))) & 1;
        if (bit) {
            ones++;
        } else {
            ones = 0;
        }

        if (bit)
            out[out_bit / 8] |= (1 << (7 - (out_bit % 8)));
        out_bit++;

        if (ones == 5) {
            out_bit++;   // вставляем 0
            ones = 0;
        }
    }
    return out_bit;
}

/**
 * Быстрый реверс битов в одном байте
 */
static uint8_t reverse_bits(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

/**
 * добавляет один бит в битовый буфер, упакованный побайтово
 */
static inline void put_bit(uint8_t *buf, size_t bit_pos, bool bit)
{
    if (bit) {
        buf[bit_pos / 8] |= (1 << (7 - (bit_pos % 8)));
    }
}

/**
 * добавляет AIS преамбулу 24 бита 010101010101010101010101
 */
static size_t append_ais_preamble(uint8_t *buf, size_t bit_pos)
{
    // AIS preamble = 24 bits: 010101010101010101010101 (LSB-first on air)
    for (int i = 0; i < 24; i++) {
        bool bit = (i & 1) ? 1 : 0;   // 0,1,0,1,...
        put_bit(buf, bit_pos++, bit);
    }
    return bit_pos;
}

/**
 * добавляет флаг HDLC 0x7E к битовому буферу, упакованному побайтово
 */
static size_t append_hdlc_flag(uint8_t *buf, size_t bit_pos)
{
    // HDLC flag: 0b01111110 (MSB first)
    const uint8_t flag = 0x7E;

    for (int i = 0; i < 8; i++) {
        bool bit = (flag >> (7 - i)) & 1;
        put_bit(buf, bit_pos, bit);
        bit_pos++;
    }
    return bit_pos;
}



/* ===================== NRZI ===================== */

static void nrzi_encode(uint8_t *data, size_t bits)
{
    bool level = 1;

    for (size_t i = 0; i < bits; i++) {
        bool bit = (data[i / 8] >> (7 - (i % 8))) & 1;
        if (bit == 0)
            level = !level;

        if (level)
            data[i / 8] |= (1 << (7 - (i % 8)));
        else
            data[i / 8] &= ~(1 << (7 - (i % 8)));
    }
}

/* ===================== SX1278 CONFIG ===================== */

static void sx1278_set_output_power_dbm(int8_t power_dbm)
{
    /* Аппаратные ограничения SX1278 (PA_BOOST, без high power) */
    if (power_dbm < 2)  power_dbm = 2;
    if (power_dbm > 17) power_dbm = 17;

    uint8_t output_power = (uint8_t)(power_dbm - 2) & 0x0F;

    uint8_t pa_config =
        SX1278_PA_SELECT_BOOST |   // PA_BOOST
        SX1278_MAX_POWER       |   // максимальный MaxPower
        output_power;              // требуемый уровень

    sx1278_write(REG_PA_CONFIG, pa_config);


    ESP_LOGI(TAG,
             "SX1278 TX power set: %d dBm (%.3f mW), RegPaConfig=0x%02X",
             power_dbm,
             pow(10.0, power_dbm / 10.0),///* Перевод dBm → mW: P[mW] = 10^(P[dBm] / 10)
             pa_config);
}


static void sx1278_init_tx(void)
{
    sx1278_write(REG_OP_MODE, MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(2));
    
    sx1278_write(REG_OP_MODE, MODE_STDBY);

    // Включаем Packet Mode
    sx1278_write(REG_PACKET_CONFIG_2, PACKET_MODE_ENABLE);

    // FIFO threshold
    sx1278_write(REG_FIFO_THRESH, FIFO_TX_START_COND_FULL | MAX_PACKET_LEN);


    // Bitrate = 32 MHz / 9600 = 3333 = 0x0D05
    const uint16_t bitrate_reg = XTAL_FREQ_HZ / AIS_BITRATE;
    sx1278_write(REG_BITRATE_MSB, (bitrate_reg >> 8) & 0xFF);
    sx1278_write(REG_BITRATE_LSB, bitrate_reg & 0xFF);
    ESP_LOGI(TAG, "Set reg bitrate: MSB=0x%02X, LSB=0x%02X", 
                                ((bitrate_reg >> 8) & 0xFF), 
                                (bitrate_reg & 0xFF));

    // --- Девиация частоты, амплитудное (не размах, не peak-to-peak) значение ±Δf ---
    const uint16_t fdev_reg = (uint16_t)(AIS_FDEV_HZ / FRF_STEP_HZ);
    sx1278_write(REG_FDEV_MSB, (fdev_reg >> 8) & 0xFF);
    sx1278_write(REG_FDEV_LSB, fdev_reg & 0xFF);
    ESP_LOGI(TAG, "Set reg frequency deviation: MSB=0x%02X, LSB=0x%02X", 
                                ((fdev_reg >> 8) & 0xFF), 
                                (fdev_reg & 0xFF));

    // --- Длительность RAMP и GMSK параметр BT Гауссовского фильтра ---
    // BT = 0.3 (AIS compliant)
    sx1278_write(REG_PA_RAMP,
                 SX1278_PA_RAMP_1MS | SX1278_GMSK_BT_0_3);

    ESP_LOGI(TAG,
             "AIS GMSK: bitrate=%d bps, fdev=±%d Hz (reg=0x%02X), Reg Ramp&BT=0x%02X",
             AIS_BITRATE, 
             AIS_FDEV_HZ, 
             fdev_reg,
             (SX1278_PA_RAMP_1MS | SX1278_GMSK_BT_0_3)
            );


    // Установка частоты несущей RF_FREQUENCY_HZ (162 MHz)
    // FRF = 162e6 / (32e6 / 2^19) ≈ 0xA20000
    uint32_t frf = RF_FREQUENCY_HZ / FRF_STEP_HZ;
    sx1278_write(REG_FRF_MSB, (frf >> 16) & 0xFF);
    sx1278_write(REG_FRF_MID, (frf >> 8)  & 0xFF);
    sx1278_write(REG_FRF_LSB,  frf        & 0xFF);
    ESP_LOGI(TAG, "Set FR frequency=%.3f MHz, reg=0x%" PRIX32, RF_FREQUENCY_HZ/1000000.0, frf);

    // устанавливаем выходную мощность сигнала
    sx1278_set_output_power_dbm(RF_POWER_DBM);

    ESP_LOGI(TAG, "Init SX1278 complete");
}



/* ===================== SEND PACKET ===================== */
static void sx1278_send_packet(const uint8_t *data, size_t length)
{
    ESP_LOGI(TAG, "Clear FIFO");
    // Очистка IRQ
    sx1278_write(REG_IRQ_FLAGS_2, 0xFF);

    // Payload length
    sx1278_write(REG_PAYLOAD_LENGTH, length);

    // FIFO
    for (size_t i = 0; i < length; i++) {
        sx1278_write(REG_FIFO, data[i]);
    }

    // TX
    uint8_t op = sx1278_read(REG_OP_MODE);
    op = (op & ~0x07) | MODE_TX;

    ESP_LOGI(TAG, "Tx mode");
    sx1278_write(REG_OP_MODE, op);

    // Ждём PacketSent
    const TickType_t timeout = pdMS_TO_TICKS(100);  // 100 мс более чем достаточно
    TickType_t start = xTaskGetTickCount();

    while (!(sx1278_read(REG_IRQ_FLAGS_2) & SX1278_IRQ2_PACKET_SENT)) {
        if ((xTaskGetTickCount() - start) > timeout) {
            ESP_LOGE(TAG, "TX timeout, forcing reset");
            //сбрасываем радиочип
            sx1278_reset();
            sx1278_init_tx();
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // sleep&Standby
    ESP_LOGI(TAG, "Sent packet complete, sleep&standby mode");
    
    //важно сначала погрузить в сон, если не делать чип может зависать
    op = (op & ~0x07) | MODE_SLEEP;
    sx1278_write(REG_OP_MODE, op);
    vTaskDelay(pdMS_TO_TICKS(1));

    op = (op & ~0x07) | MODE_STDBY;
    sx1278_write(REG_OP_MODE, op);
    vTaskDelay(pdMS_TO_TICKS(1));
}





/* ===================== MAIN ===================== */

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MISO,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NSS,
        .queue_size = 1
    };

    spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(VSPI_HOST, &devcfg, &spi);

    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    sx1278_reset();

    uint8_t ver = sx1278_read(REG_VERSION);
    ESP_LOGI(TAG, "SX1278 RegVersion = 0x%02X (expected 0x12)", ver);

    sx1278_init_tx();

    uint8_t payload[2] = {0x00, 0xFF};// {0xFD, 0xFA};
    uint16_t crc = ais_crc16(payload, 2);

    ESP_LOGI(TAG, "HDLC CRC 0x%02X", crc);

    uint8_t hdlc_bytes[6];
    size_t hdlc_len = 0;

    hdlc_bytes[hdlc_len++] = reverse_bits(payload[0]);
    hdlc_bytes[hdlc_len++] = reverse_bits(payload[1]);
    hdlc_bytes[hdlc_len++] = reverse_bits(crc & 0xFF);
    hdlc_bytes[hdlc_len++] = reverse_bits(crc >> 8);

    ESP_LOG_BUFFER_HEX(TAG, hdlc_bytes, hdlc_len);

    uint8_t stuffed_bits[64] = {0};
    size_t stuffed_bits_len = bit_stuff(
        hdlc_bytes,
        hdlc_len * 8,
        stuffed_bits
    );

    ESP_LOG_BUFFER_HEX(TAG, stuffed_bits, (stuffed_bits_len + 7)/8);

    // теперь добавляем преамбулу и флаги HDLC ПОСЛЕ bit stuffing
    size_t tx_bits = 0;
    uint8_t tx_buf[80] = {0};
    
    // AIS preamble (24 bits)
    tx_bits = append_ais_preamble(tx_buf, tx_bits);
    
    // стартовый флаг
    tx_bits = append_hdlc_flag(tx_buf, tx_bits);
    ESP_LOG_BUFFER_HEX(TAG, tx_buf, tx_bits/8);
    ESP_LOGI(TAG, "tx_bits=%d, stuffed_bits_len=%d", tx_bits, stuffed_bits_len);

    // полезные данные + CRC (уже stuffed и LSB)
    for (size_t i = 0; i < stuffed_bits_len; i++) {
        bool bit = (stuffed_bits[i / 8] >> (7 - (i % 8))) & 1;
        put_bit(tx_buf, tx_bits++, bit);
    }

    ESP_LOG_BUFFER_HEX(TAG, tx_buf, (tx_bits+7)/8);
    ESP_LOGI(TAG, "tx_bits=%d", tx_bits);

    // конечный флаг
    tx_bits = append_hdlc_flag(tx_buf, tx_bits);

    ESP_LOG_BUFFER_HEX(TAG, tx_buf, (tx_bits+7)/8);
    ESP_LOGI(TAG, "Pre NRZI: tx_bits=%d", tx_bits);

    // NRZI поверх всего битового потока
    nrzi_encode(tx_buf, tx_bits);

    // сколько байт реально передаём в SX1278
    size_t tx_len = (tx_bits + 7) / 8;

    ESP_LOG_BUFFER_HEX(TAG, tx_buf, tx_len);
    ESP_LOGI(TAG, "Post NRZI: tx_len=%d", tx_len);

    while(true) {
        sx1278_send_packet(tx_buf, tx_len);
        ESP_LOGI(TAG, "HDLC packet transmitted");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
