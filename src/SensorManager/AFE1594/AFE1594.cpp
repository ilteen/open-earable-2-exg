#include "AFE1594.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(AFE1594, LOG_LEVEL_DBG);

namespace reg {
constexpr uint16_t CONFIG_SPI1 = 0x000;
constexpr uint16_t PROG_TOP_CONTROLS = 0x002;
constexpr uint16_t PDN_CONTROLS = 0x003;
constexpr uint16_t ENABLE_CONTROLS_REG1 = 0x004;
constexpr uint16_t ENABLE_CONTROLS_REG2 = 0x005;
constexpr uint16_t PROG_SYNC = 0x008;
constexpr uint16_t PROG_CLKGEN = 0x009;
constexpr uint16_t PROG_REF = 0x040;
constexpr uint16_t PROG_RLD_REG1 = 0x047;
constexpr uint16_t PROG_RLD_REG2 = 0x048;
constexpr uint16_t INA_ECG_CH1 = 0x0C3;
constexpr uint16_t INA_ECG_CH2 = 0x0C4;
constexpr uint16_t PROG_ECG_DIG = 0x0CB;
constexpr uint16_t PROG_RBIAS_DC_LOD = 0x100;
constexpr uint16_t SEL_RBIAS_CONN = 0x101;
constexpr uint16_t SEL_AC_LEAD_BIAS_CONN = 0x106;
constexpr uint16_t PROG_MAIN_FIFO = 0x182;
constexpr uint16_t FORMAT_MAIN_FIFO = 0x184;
constexpr uint16_t READ_POINTER_MAIN_FIFO = 0x210;
constexpr uint16_t DESIGN_ID = 0x220;
constexpr uint16_t MAIN_FIFO = 0x3FF;
} // namespace reg

namespace bit {
constexpr uint32_t GLOBAL_SOFTWARE_RESET = 1UL << 23;
constexpr uint32_t SPI_REGISTER_READ = 1UL << 0;
constexpr uint32_t RESET_SYNC_COUNTER = 1UL << 22;
constexpr uint32_t RESET_FIRST_SYNC = 1UL << 21;
constexpr uint32_t GENERATE_SYNC = 1UL << 1;
constexpr uint32_t ENABLE_ECG_CH1 = 1UL << 0;
constexpr uint32_t ENABLE_ECG_CH2 = 1UL << 1;
constexpr uint32_t ENABLE_RLD_AMP = 1UL << 6;
constexpr uint32_t ENABLE_SHIELD_DRIVER = 1UL << 5;
constexpr uint32_t ENABLE_CM_IN = 1UL << 4;
constexpr uint32_t ENABLE_ECG_2CH_AVG = 1UL << 23;
constexpr uint32_t ECG_DATA_RATE_MASK = 0x0FUL << 4;
constexpr uint32_t ENABLE_MAIN_FIFO = 1UL << 0;
constexpr uint32_t MASK_ECG_CH2_TO_CH4 = (1UL << 3) | (1UL << 4) | (1UL << 5);
} // namespace bit

namespace rld {
/* TI example configuration: electrode average, RLD_REF, 50 kOhm RIN. */
constexpr uint32_t PROG_RLD_REG1_50K = 1UL << 10;
/* Average E1, E2, E3, and E4 into CM_IN. */
constexpr uint32_t AVERAGE_E1_TO_E4 = 0x00000FUL;
/* SEL_AC_LEAD_BIAS_CONN_E2 = 3 routes RLD_IN to the E2 pin. */
constexpr uint32_t ROUTE_RLD_IN_TO_E2 = 3UL << 2;
constexpr uint32_t ENABLE_CONTROLS =
    bit::ENABLE_RLD_AMP | bit::ENABLE_SHIELD_DRIVER | bit::ENABLE_CM_IN;
} // namespace rld

namespace electrode_bias {
/*
 * RBIAS code 3 selects 7.5 MOhm. The bias-level fields stay at their reset
 * values: LVL_HI=0.95*AVDD and LVL_LO=0.05*AVDD.
 *
 * The lower RBIAS value is deliberate for two dry electrodes: it makes the
 * body common-mode bias less sensitive to electrode/contact leakage while
 * keeping the worst-case current through both bias resistors near 200 nA at
 * AVDD=3.3 V.
 */
constexpr uint32_t PROG_7M5_DEFAULT_LEVELS = 3UL << 18;
/* E1 -> LVL_HI (2), E2 -> LVL_LO (1); all other electrodes open. */
constexpr uint32_t E1_HIGH_E2_LOW = 0x000006;
} // namespace electrode_bias

bool AFE1594::begin(uint8_t chip_address, const BusPins &pins)
{
    pins_ = pins;
    chip_address_ = chip_address & 0x03U;

    if (!device_is_ready(pins_.gpio)) {
        LOG_ERR("GPIO device for software SPI is not ready");
        return false;
    }

    LOG_INF("Software SPI GPIO=%s CS=P0.%u SCK=P0.%u MOSI=P0.%u MISO=P0.%u, mode 0",
            pins_.gpio->name, pins_.cs, pins_.sck, pins_.mosi, pins_.miso);

    if (gpio_pin_configure(pins_.gpio, pins_.cs, GPIO_OUTPUT_HIGH) != 0 ||
        gpio_pin_configure(pins_.gpio, pins_.sck, GPIO_OUTPUT_LOW) != 0 ||
        gpio_pin_configure(pins_.gpio, pins_.mosi, GPIO_OUTPUT_LOW) != 0 ||
        gpio_pin_configure(pins_.gpio, pins_.miso, GPIO_INPUT) != 0) {
        LOG_ERR("Failed to configure AFE1594 software SPI pins");
        return false;
    }

    if (!reset()) {
        return false;
    }

    /* Match Erik's known-good ESP configuration: internal clock and 1x ADC. */
    if (!writeRegister(reg::PROG_CLKGEN, 0x000001UL)) {
        return false;
    }
    k_msleep(200);
    if (!writeRegister(reg::PROG_REF, 0x000002UL) ||
        !writeRegister(reg::PDN_CONTROLS, 0x000000UL)) {
        return false;
    }
    k_msleep(100);

    if (!updateRegister(reg::PROG_TOP_CONTROLS, 0x03UL, 0) ||
        !configureMainFifo()) {
        return false;
    }

    const uint8_t design_id = readDesignId();
    if (design_id != 0x02U) {
        LOG_ERR("Unexpected AFE1594 design ID: 0x%02x", design_id);
        return false;
    }

    initialized_ = true;
    LOG_INF("AFE1594 detected (design ID 0x%02x)", design_id);
    return true;
}

bool AFE1594::configureAveragedChannels(ChannelGain gain)
{
    const uint32_t channel_1 =
        (static_cast<uint32_t>(gain) << 8) |
        (static_cast<uint32_t>(ChannelInput::E1) << 4) |
        static_cast<uint32_t>(ChannelInput::E3);
    const uint32_t channel_2 =
        (static_cast<uint32_t>(gain) << 8) |
        (static_cast<uint32_t>(ChannelInput::E2) << 4) |
        static_cast<uint32_t>(ChannelInput::E4);
    const uint32_t channel_enable = bit::ENABLE_ECG_CH1 | bit::ENABLE_ECG_CH2;

    if (!writeRegister(reg::INA_ECG_CH1, channel_1) ||
        !writeRegister(reg::INA_ECG_CH2, channel_2) ||
        !updateRegister(reg::ENABLE_CONTROLS_REG2, channel_enable,
                        channel_enable) ||
        !updateRegister(reg::PROG_ECG_DIG, bit::ENABLE_ECG_2CH_AVG,
                        bit::ENABLE_ECG_2CH_AVG) ||
        !configureMainFifo()) {
        return false;
    }

    const uint32_t read_channel_1 = readRegister(reg::INA_ECG_CH1);
    const uint32_t read_channel_2 = readRegister(reg::INA_ECG_CH2);
    const uint32_t read_enable = readRegister(reg::ENABLE_CONTROLS_REG2);
    const uint32_t read_digital = readRegister(reg::PROG_ECG_DIG);
    if (read_channel_1 != channel_1 || read_channel_2 != channel_2 ||
        (read_enable & channel_enable) != channel_enable ||
        (read_digital & bit::ENABLE_ECG_2CH_AVG) == 0U) {
        LOG_ERR("Two-channel average readback failed: ch1=0x%06x ch2=0x%06x en=0x%06x dig=0x%06x",
                (unsigned int)read_channel_1, (unsigned int)read_channel_2,
                (unsigned int)read_enable, (unsigned int)read_digital);
        return false;
    }

    LOG_INF("ECG average enabled: CH1=E1-E3, CH2=E2-E4, gain %.2f; CH2 FIFO word masked",
            (double)gainValue(gain));
    return true;
}

bool AFE1594::configurePatientBias(PatientBiasMode mode)
{
    /* Begin both modes with every bias/drive path disconnected. */
    if (!writeRegister(reg::ENABLE_CONTROLS_REG1, 0) ||
        !writeRegister(reg::PROG_RLD_REG1, 0) ||
        !writeRegister(reg::PROG_RLD_REG2, 0) ||
        !writeRegister(reg::SEL_AC_LEAD_BIAS_CONN, 0) ||
        !writeRegister(reg::SEL_RBIAS_CONN, 0)) {
        return false;
    }

    if (mode != PatientBiasMode::PassiveTwoElectrode) {
        /*
         * RLD_OUT reaches RLD_IN through the board's protection/feedback
         * network. DedicatedRld leaves register 106 clear for an electrode on
         * the RLD_IN pad. SharedE2Rld routes RLD_IN internally onto E2 instead.
         */
        const uint32_t route = mode == PatientBiasMode::SharedE2Rld
                                   ? rld::ROUTE_RLD_IN_TO_E2
                                   : 0U;
        if (!writeRegister(reg::PROG_RBIAS_DC_LOD, 0) ||
            !writeRegister(reg::PROG_RLD_REG1, rld::PROG_RLD_REG1_50K) ||
            !writeRegister(reg::PROG_RLD_REG2, rld::AVERAGE_E1_TO_E4) ||
            !writeRegister(reg::SEL_AC_LEAD_BIAS_CONN, route) ||
            !writeRegister(reg::ENABLE_CONTROLS_REG1, rld::ENABLE_CONTROLS)) {
            return false;
        }

        const uint32_t controls = readRegister(reg::ENABLE_CONTROLS_REG1);
        const uint32_t rld_1 = readRegister(reg::PROG_RLD_REG1);
        const uint32_t rld_2 = readRegister(reg::PROG_RLD_REG2);
        const uint32_t read_route = readRegister(reg::SEL_AC_LEAD_BIAS_CONN);
        const uint32_t passive = readRegister(reg::SEL_RBIAS_CONN);
        if (controls != rld::ENABLE_CONTROLS ||
            rld_1 != rld::PROG_RLD_REG1_50K ||
            rld_2 != rld::AVERAGE_E1_TO_E4 || read_route != route ||
            passive != 0U) {
            LOG_ERR("RLD readback failed: en=0x%06x rld1=0x%06x rld2=0x%06x route=0x%06x rbias=0x%06x",
                    (unsigned int)controls, (unsigned int)rld_1,
                    (unsigned int)rld_2, (unsigned int)read_route,
                    (unsigned int)passive);
            return false;
        }

        LOG_INF("RLD on: CM=average(E1..E4), RIN=50 kOhm, RLD_REF, %s",
                mode == PatientBiasMode::SharedE2Rld
                    ? "shared E2 reference electrode"
                    : "dedicated RLD_IN electrode");
        return true;
    }

    if (!writeRegister(reg::PROG_RBIAS_DC_LOD,
                       electrode_bias::PROG_7M5_DEFAULT_LEVELS) ||
        !writeRegister(reg::SEL_RBIAS_CONN,
                       electrode_bias::E1_HIGH_E2_LOW)) {
        return false;
    }

    const uint32_t programmed = readRegister(reg::PROG_RBIAS_DC_LOD);
    const uint32_t selected = readRegister(reg::SEL_RBIAS_CONN);
    if (programmed != electrode_bias::PROG_7M5_DEFAULT_LEVELS ||
        selected != electrode_bias::E1_HIGH_E2_LOW) {
        LOG_ERR("Passive bias readback failed: prog=0x%06x sel=0x%06x",
                (unsigned int)programmed, (unsigned int)selected);
        return false;
    }

    LOG_INF("RLD off: E1->0.95*AVDD, E2->0.05*AVDD via 7.5 MOhm");
    return true;
}

bool AFE1594::setDataRate(uint8_t selector)
{
    if (selector > 11U || selector == 6U || selector == 7U) {
        return false;
    }
    return updateRegister(reg::PROG_ECG_DIG, bit::ECG_DATA_RATE_MASK,
                          static_cast<uint32_t>(selector & 0x0FU) << 4);
}

bool AFE1594::startStreaming()
{
    return initialized_ &&
           updateRegister(reg::PROG_MAIN_FIFO, bit::ENABLE_MAIN_FIFO, 0) &&
           triggerSync() &&
           updateRegister(reg::PROG_MAIN_FIFO, bit::ENABLE_MAIN_FIFO,
                          bit::ENABLE_MAIN_FIFO);
}

bool AFE1594::stopStreaming()
{
    return !initialized_ ||
           updateRegister(reg::PROG_MAIN_FIFO, bit::ENABLE_MAIN_FIFO, 0);
}

size_t AFE1594::readSamples(int32_t *samples, size_t capacity, bool *overflow)
{
    if (overflow != nullptr) {
        *overflow = false;
    }
    if (!initialized_ || samples == nullptr || capacity == 0U) {
        return 0;
    }

    const uint32_t pointer = readRegister(reg::READ_POINTER_MAIN_FIFO);
    if (overflow != nullptr) {
        *overflow = (pointer & FIFO_REGISTER_OVERFLOW) != 0;
    }
    if ((pointer & FIFO_REGISTER_EMPTY) != 0) {
        return 0;
    }

    size_t count = pointer & FIFO_REGISTER_WORD_COUNT;
    if (count > capacity) {
        count = capacity;
    }
    if (count > MAX_FIFO_BURST_WORDS) {
        count = MAX_FIFO_BURST_WORDS;
    }
    if (count == 0U) {
        return 0;
    }

    /*
     * Keep the complete FIFO cycle in one SPI transaction. The first word is
     * the AFE read-start/status word; subsequent words are sample data.
     * SPI_REG_READ is intentionally left clear for FIFO reads (datasheet
     * section 9.3.19.1).
     */
    constexpr size_t address_bytes = 2;
    constexpr size_t word_bytes = 3;
    uint8_t tx[address_bytes + word_bytes * (MAX_FIFO_BURST_WORDS + 1)] = {};
    uint8_t rx[sizeof(tx)] = {};
    const uint16_t address = addressWord(reg::MAIN_FIFO);
    tx[0] = static_cast<uint8_t>(address >> 8);
    tx[1] = static_cast<uint8_t>(address);
    const size_t transfer_length = address_bytes + word_bytes * (count + 1U);

    select();
    const bool transfer_ok = transfer(tx, rx, transfer_length);
    deselect();
    if (!transfer_ok) {
        return 0;
    }

    const uint32_t read_start =
        (static_cast<uint32_t>(rx[2]) << 16) |
        (static_cast<uint32_t>(rx[3]) << 8) |
        static_cast<uint32_t>(rx[4]);
    if (overflow != nullptr && (read_start & FIFO_READ_START_OVERFLOW) != 0) {
        *overflow = true;
    }
    const size_t available = read_start & FIFO_READ_START_WORD_COUNT;
    if ((read_start & FIFO_READ_START_EMPTY) != 0 || available < count) {
        LOG_WRN("AFE1594 FIFO read-start mismatch: requested=%zu available=%zu status=0x%06x",
                count, available, (unsigned int)read_start);
        return 0;
    }

    for (size_t i = 0; i < count; ++i) {
        const size_t offset = address_bytes + word_bytes * (i + 1U);
        const uint32_t raw =
            (static_cast<uint32_t>(rx[offset]) << 16) |
            (static_cast<uint32_t>(rx[offset + 1U]) << 8) |
            static_cast<uint32_t>(rx[offset + 2U]);
        samples[i] = signExtend24(raw);
    }
    return count;
}

uint8_t AFE1594::readDesignId()
{
    last_design_id_read_ = readRegister(reg::DESIGN_ID);
    return static_cast<uint8_t>(last_design_id_read_ & 0xFFU);
}

uint32_t AFE1594::lastDesignIdRead() const { return last_design_id_read_; }
int AFE1594::lastSpiError() const { return last_spi_error_; }

float AFE1594::gainValue(ChannelGain gain)
{
    switch (gain) {
    case ChannelGain::Gain1p25: return 1.25f;
    case ChannelGain::Gain2: return 2.0f;
    case ChannelGain::Gain3: return 3.0f;
    case ChannelGain::Gain4: return 4.0f;
    case ChannelGain::Gain5: return 5.0f;
    case ChannelGain::Gain6p5: return 6.5f;
    case ChannelGain::Gain9: return 9.0f;
    }
    return 1.0f;
}

uint8_t AFE1594::transferByte(uint8_t value)
{
    uint8_t result = 0;

    /* Mode 0: present MOSI while SCK is low, sample MISO after rising edge. */
    for (int bit_index = 7; bit_index >= 0; --bit_index) {
        int ret = gpio_pin_set(pins_.gpio, pins_.mosi,
                               (value >> bit_index) & 0x01U);
        if (ret != 0 && last_spi_error_ == 0) {
            last_spi_error_ = ret;
        }
        k_busy_wait(1);

        ret = gpio_pin_set(pins_.gpio, pins_.sck, 1);
        if (ret != 0 && last_spi_error_ == 0) {
            last_spi_error_ = ret;
        }
        k_busy_wait(1);

        const int level = gpio_pin_get(pins_.gpio, pins_.miso);
        if (level < 0) {
            if (last_spi_error_ == 0) {
                last_spi_error_ = level;
            }
        } else {
            result |= static_cast<uint8_t>(level & 0x01) << bit_index;
        }

        ret = gpio_pin_set(pins_.gpio, pins_.sck, 0);
        if (ret != 0 && last_spi_error_ == 0) {
            last_spi_error_ = ret;
        }
    }
    return result;
}

bool AFE1594::transfer(const uint8_t *tx_data, uint8_t *rx_data, size_t length)
{
    for (size_t i = 0; i < length; ++i) {
        rx_data[i] = transferByte(tx_data[i]);
    }
    return last_spi_error_ == 0;
}

void AFE1594::select()
{
    const int ret = gpio_pin_set(pins_.gpio, pins_.cs, 0);
    if (ret != 0 && last_spi_error_ == 0) {
        last_spi_error_ = ret;
    }
}

void AFE1594::deselect()
{
    const int ret = gpio_pin_set(pins_.gpio, pins_.cs, 1);
    if (ret != 0 && last_spi_error_ == 0) {
        last_spi_error_ = ret;
    }
}

uint16_t AFE1594::addressWord(uint16_t reg_address) const
{
    return (static_cast<uint16_t>(chip_address_) << 14) |
           (reg_address & 0x3FFFU);
}

bool AFE1594::writeFrame(uint16_t reg_address, uint32_t value)
{
    const uint16_t address = addressWord(reg_address);
    value &= MASK_24;
    last_spi_error_ = 0;

    select();
    transferByte(static_cast<uint8_t>(address >> 8));
    transferByte(static_cast<uint8_t>(address));
    transferByte(static_cast<uint8_t>(value >> 16));
    transferByte(static_cast<uint8_t>(value >> 8));
    transferByte(static_cast<uint8_t>(value));
    deselect();
    k_busy_wait(5);
    return last_spi_error_ == 0;
}

uint32_t AFE1594::readFrame(uint16_t reg_address)
{
    const uint16_t address = addressWord(reg_address);
    last_spi_error_ = 0;

    select();
    transferByte(static_cast<uint8_t>(address >> 8));
    transferByte(static_cast<uint8_t>(address));
    transferByte(0);
    transferByte(0);
    transferByte(0);
    deselect();
    k_busy_wait(5);
    if (last_spi_error_ != 0) {
        return 0;
    }

    last_spi_error_ = 0;
    select();
    transferByte(static_cast<uint8_t>(address >> 8));
    transferByte(static_cast<uint8_t>(address));
    const uint32_t value =
        (static_cast<uint32_t>(transferByte(0)) << 16) |
        (static_cast<uint32_t>(transferByte(0)) << 8) |
        static_cast<uint32_t>(transferByte(0));
    deselect();
    if (last_spi_error_ != 0) {
        return 0;
    }
    return value & MASK_24;
}

bool AFE1594::writeRegister(uint16_t reg_address, uint32_t value)
{
    if (!writeFrame(reg_address, value)) {
        LOG_ERR("SPI write failed: reg=0x%03x value=0x%06x error=%d",
                reg_address, static_cast<unsigned int>(value & MASK_24),
                last_spi_error_);
        return false;
    }
    LOG_DBG("SPI write reg 0x%03x = 0x%06x", reg_address,
            static_cast<unsigned int>(value & MASK_24));
    if (reg_address == reg::CONFIG_SPI1) {
        config_spi1_shadow_ = value & MASK_24;
    }
    return true;
}

uint32_t AFE1594::readRegister(uint16_t reg_address)
{
    if (!setReadMode(true)) {
        return 0;
    }
    const uint32_t value = readFrame(reg_address);
    setReadMode(false);
    if (reg_address != reg::READ_POINTER_MAIN_FIFO) {
        LOG_DBG("SPI read  reg 0x%03x = 0x%06x", reg_address,
                static_cast<unsigned int>(value));
    }
    return value;
}

bool AFE1594::updateRegister(uint16_t reg_address, uint32_t mask, uint32_t value)
{
    const uint32_t current = readRegister(reg_address);
    return writeRegister(reg_address, (current & ~mask) | (value & mask));
}

bool AFE1594::setReadMode(bool enabled)
{
    uint32_t value = config_spi1_shadow_;
    if (enabled) {
        value |= bit::SPI_REGISTER_READ;
    } else {
        value &= ~bit::SPI_REGISTER_READ;
    }
    return writeFrame(reg::CONFIG_SPI1, value);
}

bool AFE1594::reset()
{
    if (!writeRegister(reg::CONFIG_SPI1, bit::GLOBAL_SOFTWARE_RESET)) {
        return false;
    }
    k_msleep(10);
    config_spi1_shadow_ = 0;

    return writeRegister(reg::PROG_REF, 0x000002UL);
}

bool AFE1594::configureMainFifo()
{
    /* One channel word per frame; no tag, timestamp, pace, or RLD data. */
    return writeRegister(reg::PROG_MAIN_FIFO, 4UL << 4) &&
           writeRegister(reg::FORMAT_MAIN_FIFO, bit::MASK_ECG_CH2_TO_CH4);
}

bool AFE1594::triggerSync()
{
    if (!writeRegister(reg::PROG_SYNC, 0)) {
        return false;
    }
    k_msleep(10);
    if (!writeRegister(reg::PROG_SYNC,
                       bit::RESET_SYNC_COUNTER | bit::RESET_FIRST_SYNC)) {
        return false;
    }
    k_msleep(10);
    if (!writeRegister(reg::PROG_SYNC, 0)) {
        return false;
    }
    k_msleep(10);
    if (!writeRegister(reg::PROG_SYNC, bit::GENERATE_SYNC)) {
        return false;
    }
    k_msleep(100);
    return true;
}

int32_t AFE1594::signExtend24(uint32_t raw)
{
    raw &= MASK_24;
    if ((raw & 0x00800000UL) != 0U) {
        raw |= 0xFF000000UL;
    }
    return static_cast<int32_t>(raw);
}
