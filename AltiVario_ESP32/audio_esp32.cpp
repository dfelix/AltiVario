#include "audio_esp32.h"
#include <driver/i2s.h>
#include <math.h>

AudioESP32::AudioESP32()
    : _piezoPin(0)
    , _ledcChannel(0)
    , _mode(0)
    , _volume(80)
    , _piezoReady(false)
    , _dacReady(false)
    , _bclkPin(0)
    , _lrclkPin(0)
    , _doutPin(0)
{
}

void AudioESP32::beginPiezo(uint8_t pin, uint8_t ledcChannel) {
    _piezoPin = pin;
    _ledcChannel = ledcChannel;
    ledcAttach(_piezoPin, 1000, 8);
    _piezoReady = true;
}

void AudioESP32::beginDAC(uint8_t bclkPin, uint8_t lrclkPin, uint8_t doutPin) {
    _bclkPin = bclkPin;
    _lrclkPin = lrclkPin;
    _doutPin = doutPin;

    // Configure I2S for MAX98357A DAC
    // Using ESP-IDF I2S standard mode at 44100 Hz, 16-bit mono
    i2s_config_t i2s_config = {};
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    i2s_config.sample_rate = 44100;
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_config.dma_buf_count = 4;
    i2s_config.dma_buf_len = 256;
    i2s_config.use_apll = false;

    i2s_pin_config_t pin_config = {};
    pin_config.bck_io_num = _bclkPin;
    pin_config.ws_io_num = _lrclkPin;
    pin_config.data_out_num = _doutPin;
    pin_config.data_in_num = I2S_PIN_NO_CHANGE;

    if (i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL) == ESP_OK) {
        if (i2s_set_pin(I2S_NUM_0, &pin_config) == ESP_OK) {
            _dacReady = true;
        }
    }
}

void AudioESP32::play(const ToneParams& tp) {
    if (tp.frequency == 0 || _volume == 0) {
        stop();
        return;
    }

    if (_mode == 0 && _piezoReady) {
        playPiezo(tp.frequency, tp.duration);
    } else if (_mode == 1 && _dacReady) {
        playDAC(tp.frequency, tp.duration);
    }
}

void AudioESP32::stop() {
    if (_piezoReady) stopPiezo();
    if (_dacReady) stopDAC();
}

void AudioESP32::playPiezo(uint16_t freq, uint16_t duration) {
    // Set duty cycle based on volume (0-127 for 8-bit resolution)
    uint8_t duty = map(_volume, 0, 100, 0, 127);
    ledcWriteTone(_piezoPin, freq);
    ledcWrite(_piezoPin, duty);
}

void AudioESP32::stopPiezo() {
    ledcWriteTone(_piezoPin, 0);
    ledcWrite(_piezoPin, 0);
}

void AudioESP32::playDAC(uint16_t freq, uint16_t duration) {
    // Generate sine wave samples for the given frequency and duration
    const uint32_t sampleRate = 44100;
    uint32_t numSamples = (sampleRate * duration) / 1000;
    float amplitude = (_volume / 100.0f) * 32767.0f;

    // Write in chunks to avoid large allocation
    const uint16_t chunkSize = 256;
    int16_t samples[chunkSize];

    uint32_t written = 0;
    while (written < numSamples) {
        uint16_t count = min((uint32_t)chunkSize, numSamples - written);
        for (uint16_t i = 0; i < count; i++) {
            float t = (float)(written + i) / sampleRate;
            samples[i] = (int16_t)(amplitude * sinf(2.0f * M_PI * freq * t));
        }
        size_t bytesWritten;
        i2s_write(I2S_NUM_0, samples, count * sizeof(int16_t),
                  &bytesWritten, portMAX_DELAY);
        written += count;
    }
}

void AudioESP32::stopDAC() {
    i2s_zero_dma_buffer(I2S_NUM_0);
}
