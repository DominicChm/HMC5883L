
#ifndef ASYNCWIREOPERATIONS_H
#define ASYNCWIREOPERATIONS_H

#include <Arduino.h>
#include <Wire.h>

// Class to help with I2C operations. Supports both synchronous and
// asynchronous operation.
class AsyncWire {
private:
    TwoWire *wire;
    int finishValue = 0;
    int *finishFlag = 0;

    uint8_t *targetBuffer;
    size_t length;
    size_t bufferHead;

    int targetAddress = 0x00;
    int state = 0;

    const static int STATE_IDLE = 0;
    const static int STATE_RECEIVE = 1;
    const static int STATE_FINALIZE = 2;

public:

    void begin(TwoWire &wire, int targetAddress) {
        this->wire = &wire;
        this->targetAddress = targetAddress;
    }

	// Read a single byte from the passed register. Blocking.
    uint8_t readSync8(uint8_t reg) {
        setRegisterPointer(reg);

        uint8_t value;
        wire->beginTransmission(targetAddress);
        wire->requestFrom(targetAddress, 1);

        while (!wire->available()) {;}
        value = (uint8_t) wire->read();

        wire->endTransmission();

        return value;
    }

	// Read a single byte from the passed register. Non-blocking.
	// Returns whether the read was sucessfully started.
	// Will read length into the passed buffer, setting *finishFlag to
	// finishValue once completed. Designed to allow interoperation 
	// with an FSM. (IE *finishFlag is state, finishValue is next state) 
    bool readAsync(uint8_t buffer[], size_t length, int *finishFlag, int finishValue) {
        if (state != STATE_IDLE) return false;

        this->length = length;
        this->targetBuffer = buffer;

        this->finishFlag = finishFlag;
        this->finishValue = finishValue;

        wire->beginTransmission(targetAddress);
        wire->requestFrom(targetAddress, length);

        bufferHead = 0;
        state = STATE_RECEIVE;

        return true;
    }

	// Reads a buffer without setting the pointer. Blocking.
    bool readSync(uint8_t *buffer, size_t length) {
        if (state != STATE_IDLE) return false;
        wire->beginTransmission(targetAddress);
        wire->requestFrom(targetAddress, length);

        bufferHead = 0;
        while (wire->available() < (int) length) {;}
        wire->readBytes(buffer, length);
        return true;
    }

	// Reads a buffer after setting the pointer. Blocking.
    bool readRegSync(uint8_t reg, uint8_t *buffer, size_t length) {
        if (state != STATE_IDLE) return false;
        setRegisterPointer(reg);

        wire->beginTransmission(targetAddress);
        wire->requestFrom(targetAddress, length);

        while (wire->available() < (int) length) {;}
        wire->readBytes(buffer, length);
        return true;
    }

	// The async FSM.
    void loop() {
        switch (state) {
            case STATE_IDLE:
                return;
            case STATE_RECEIVE: {
                if (wire->available()) {
                    targetBuffer[bufferHead] = (uint8_t) wire->read();
                    bufferHead++;
                }
                if (bufferHead >= length) {
                    state = STATE_FINALIZE;
                }
                break;
            }
            case STATE_FINALIZE: {
                wire->endTransmission();
                *finishFlag = finishValue;
                state = STATE_IDLE;
                break;
            }
            default:
                return;
        }
    }

    /*Non-asyncable ops*/
    void setRegisterPointer(uint8_t ptr) {
        wire->beginTransmission(targetAddress);
        wire->write(ptr);
        wire->endTransmission();
    }

    void writeSync8(uint8_t reg, uint8_t value) {
        wire->beginTransmission(targetAddress);
        wire->write(reg);
        wire->write(value);
        wire->endTransmission();
    }
};

#endif
