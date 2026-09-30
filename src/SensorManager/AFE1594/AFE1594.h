#ifndef OPENEARABLE_AFE1594_H
#define OPENEARABLE_AFE1594_H

#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

class AFE1594 {
public:
    static constexpr size_t ECG_CHANNEL_COUNT = 4;

    enum class ChannelInput : uint8_t {
        RldRef = 0x00,
        E1 = 0x01,
        E2 = 0x02,
        E3 = 0x03,
        E4 = 0x04,
        E5 = 0x05,
        E6 = 0x06,
        E7 = 0x07,
        E8 = 0x08,
        Wct = 0x0C,
        RldIn = 0x0E,
    };

    enum class ChannelGain : uint8_t {
        Gain1p25 = 0x02,
        Gain2 = 0x00,
        Gain3 = 0x01,
        Gain4 = 0x06,
        Gain5 = 0x07,
        Gain6p5 = 0x04,
        Gain9 = 0x05,
    };

    enum class PatientBiasMode : uint8_t {
        PassiveTwoElectrode,
        DedicatedRld,
        SharedE2Rld,
    };

    struct BusPins {
        const struct device *gpio;
        gpio_pin_t cs;
        gpio_pin_t sck;
        gpio_pin_t mosi;
        gpio_pin_t miso;
    };

    bool begin(uint8_t chip_address, const BusPins &pins);
    bool configurePatientBias(PatientBiasMode mode);
    bool configureDifferentialChannels(ChannelGain gain);
    bool setDataRate(uint8_t selector);
    bool startStreaming();
    bool stopStreaming();
    size_t readSampleFrames(int32_t (*frames)[ECG_CHANNEL_COUNT], size_t capacity,
                            bool *overflow = nullptr);

    uint8_t readDesignId();
    uint32_t lastDesignIdRead() const;
    int lastSpiError() const;
    static float gainValue(ChannelGain gain);

private:
    static constexpr uint32_t MASK_24 = 0x00FFFFFFUL;
    static constexpr uint32_t FIFO_REGISTER_OVERFLOW = 0x00008000UL;
    static constexpr uint32_t FIFO_REGISTER_EMPTY = 0x00004000UL;
    static constexpr uint32_t FIFO_REGISTER_WORD_COUNT = 0x00000FFFUL;
    static constexpr uint32_t FIFO_READ_START_OVERFLOW = 0x00800000UL;
    static constexpr uint32_t FIFO_READ_START_EMPTY = 0x00400000UL;
    static constexpr uint32_t FIFO_READ_START_WORD_COUNT = 0x00003FFFUL;
    static constexpr size_t MAX_FIFO_BURST_WORDS = 16;

    BusPins pins_{};
    uint8_t chip_address_ = 0;
    uint32_t config_spi1_shadow_ = 0;
    uint32_t last_design_id_read_ = 0;
    int last_spi_error_ = 0;
    bool initialized_ = false;

    uint8_t transferByte(uint8_t value);
    bool transfer(const uint8_t *tx_data, uint8_t *rx_data, size_t length);
    void select();
    void deselect();
    uint16_t addressWord(uint16_t reg) const;
    bool writeFrame(uint16_t reg, uint32_t value);
    uint32_t readFrame(uint16_t reg);
    bool writeRegister(uint16_t reg, uint32_t value);
    uint32_t readRegister(uint16_t reg);
    bool updateRegister(uint16_t reg, uint32_t mask, uint32_t value);
    bool setReadMode(bool enabled);
    bool reset();
    bool configureMainFifo();
    bool triggerSync();
    static int32_t signExtend24(uint32_t raw);
};

#endif
