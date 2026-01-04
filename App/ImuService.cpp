#include "ImuService.h"
#include <string.h>

static uint8_t calcChecksum(const uint8_t* data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) sum += data[i];
    return sum;
}

ImuService::ImuService(IIoPort& ioPort) : _io(ioPort) {
    resetAcc();
    resetOffsets();
    memset(_lastSentVals, 0, sizeof(_lastSentVals));
}

void ImuService::process(const SensorFrame& frame) {
    if (_io.readCommand() == 'C') { 
        enterCalibration();
    }
    if (_state == State::Calibrating || !_io.isTxBusy()) {
        accumulateData(frame);
    } else {
        // 丢弃
        return; 
    }
    switch (_state) {
        case State::Normal:      runNormalLogic();      break;
        case State::Calibrating: runCalibrationLogic(); break;
    }
}

void ImuService::runNormalLogic() {
    if (_sampleCount < 1) return;
    if (_io.isTxBusy()) return;

    calculateAverages();
    sendDataPacket();
    resetAcc();
    _framesSinceKeyframe++;
}

void ImuService::runCalibrationLogic() {
    if (_sampleCount < 1) return;
    for (int i = 0; i < SENSORS; ++i) {
        for (int a = 3; a < AXIS; ++a) { 
            _offsets[a][i] += _accumulator[a][i];
        }
    }
    _calibCount++;
    resetAcc();
    if (_calibCount >= 500) finishCalibration();
}

void ImuService::enterCalibration() {
    _state = State::Calibrating;
    _calibCount = 0;
    resetAcc();
    resetOffsets();
}

void ImuService::finishCalibration() {
    for (int i = 0; i < SENSORS; ++i) {
        for (int a = 3; a < AXIS; ++a) {
            _offsets[a][i] /= 500;
        }
    }
    _isCalibrated = true;
    _state = State::Normal;
}

void ImuService::accumulateData(const SensorFrame& frame) {
    for (int a = 0; a < AXIS; ++a) {
        for (int i = 0; i < SENSORS; ++i) {
            _accumulator[a][i] += frame.data[a][i];
        }
    }
    _sampleCount++;
}

void ImuService::calculateAverages() {
    for (int a = 0; a < AXIS; ++a) {
        for (int i = 0; i < SENSORS; ++i) {
            int32_t val = _accumulator[a][i] / _sampleCount;
            if (_isCalibrated && a >= 3) val -= _offsets[a][i];
            _currentVals[a][i] = val;
        }
    }
}

void ImuService::sendDataPacket() {
    static uint8_t txBuf[2048]; 
    size_t len = 0;

    if (shouldSendRaw()) {
        len = fillRawPacket(txBuf);
        _framesSinceKeyframe = 0;
    } else {
        len = fillDeltaPacket(txBuf);
    }
    memcpy(_lastSentVals, _currentVals, sizeof(_currentVals));

    _io.write(txBuf, len);
}

bool ImuService::shouldSendRaw() {
    if (_framesSinceKeyframe >= KEYFRAME_INTERVAL) return true;
    for (int a = 0; a < AXIS; ++a) {
        for (int i = 0; i < SENSORS; ++i) {
			if (i == 0 || i == 18 || i == 63) continue;
            int32_t diff = _currentVals[a][i] - _lastSentVals[a][i];
            if (diff < -127 || diff > 127) return true;
        }
    }
    return false;
}

void ImuService::resetAcc() {
    memset(_accumulator, 0, sizeof(_accumulator));
    _sampleCount = 0;
}
void ImuService::resetOffsets() {
    memset(_offsets, 0, sizeof(_offsets));
}

size_t ImuService::fillRawPacket(uint8_t* buf) {
    auto* head = reinterpret_cast<FrameHeader*>(buf);
    auto* body = reinterpret_cast<PayloadRaw*>(buf + sizeof(FrameHeader));

    head->magic[0] = 0xA5; head->magic[1] = 0x5A;
    head->type = FrameType::Raw16;

    for (int a = 0; a < AXIS; ++a) {
        for (int i = 0; i < SENSORS; ++i) {
            body->data[a][i] = static_cast<int16_t>(_currentVals[a][i]);
        }
    }
    size_t total = sizeof(FrameHeader) + sizeof(PayloadRaw);
    buf[total] = calcChecksum(buf, total);
    return total + 1;
}

size_t ImuService::fillDeltaPacket(uint8_t* buf) {
    auto* head = reinterpret_cast<FrameHeader*>(buf);
    auto* body = reinterpret_cast<PayloadDelta*>(buf + sizeof(FrameHeader));

    head->magic[0] = 0xA5; head->magic[1] = 0x5A;
    head->type = FrameType::Delta8;

    for (int a = 0; a < AXIS; ++a) {
        for (int i = 0; i < SENSORS; ++i) {
            int32_t diff = _currentVals[a][i] - _lastSentVals[a][i];
            body->data[a][i] = static_cast<int8_t>(diff);
        }
    }
    size_t total = sizeof(FrameHeader) + sizeof(PayloadDelta);
    buf[total] = calcChecksum(buf, total);
    return total + 1;
}