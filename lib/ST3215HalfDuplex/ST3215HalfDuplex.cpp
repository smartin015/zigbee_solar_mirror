#include "ST3215HalfDuplex.h"

#include <soc/gpio_struct.h>

SMS_STS_HalfDuplex::SMS_STS_HalfDuplex() : SMS_STS() {}

void SMS_STS_HalfDuplex::begin(int8_t pin, HardwareSerial &serial) {
  _pin = pin;
  _serial = &serial;
  _mask = (uint32_t)(1ULL << pin);
  _txMode = false;
  _txBytesWritten = 0;

  // Keep the base-class serial pointer valid as well.
  pSerial = &serial;

  // Map both UART RX and TX onto the same GPIO. The ESP32 UART driver
  // explicitly supports this single-pin (rx == tx) configuration.
  serial.setPins(pin, pin);
  serial.begin(1000000, SERIAL_8N1);

  // While the TX pad driver is tri-stated (RX mode) the internal pull-up
  // keeps the shared line in the UART idle (high) state.
  gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLUP_ONLY);

  setRx();
}

int SMS_STS_HalfDuplex::changeId(uint8_t currentId, uint8_t newId) {
  // 0xFE is the broadcast ID and cannot be used as a normal servo ID.
  if (newId > 0xFD) {
    return -1;
  }
  if (currentId == newId) {
    return 1;
  }

  // EEPROM writes are protected: unlock the protection first. The write is
  // the important part; a missing ack on the single-pin link does not tell
  // us whether the command arrived, so verify with Ping(newId) below.
  int unlockAck = unLockEprom(currentId);
  delay(5);

  // Write the new ID with a broadcast frame, exactly as the Waveshare
  // protocol manual's "set ID" example does. A broadcast write returns no
  // acknowledgement, which avoids the ambiguity of an ack that may carry
  // the old or the new ID.
  writeByte(0xFE, SMS_STS_ID, newId);

  // Verify the new ID is live. The EEPROM save can keep the servo busy for
  // a few milliseconds, so give it a couple of short retries.
  int pingOk = -1;
  for (uint8_t attempt = 0; attempt < 5 && pingOk < 0; attempt++) {
    delay(20);
    pingOk = Ping(newId);
  }

  if (pingOk < 0) {
    // If the unlock ack was also missing the RX path probably never saw a
    // response; otherwise the unlock succeeded but the ID write failed.
    return unlockAck ? -3 : -2;
  }

  // Re-lock the EEPROM using the new ID.
  if (!LockEprom(newId)) {
    return -4;
  }
  return 1;
}

void SMS_STS_HalfDuplex::setTx() {
  if (_txMode) {
    return;
  }

  // Re-enable the pad output driver. The UART TX signal is still routed to
  // this pad by Serial.setPins(), so it drives the servo DATA line.
  GPIO.enable_w1ts.val = _mask;
  _txMode = true;
}

void SMS_STS_HalfDuplex::setRx() {
  if (!_txMode) {
    return;
  }

  // Tri-state the pad output driver so the servo can drive the shared line
  // while it sends its response. RX remains routed to the same pad.
  GPIO.enable_w1tc.val = _mask;
  _txMode = false;
}

int SMS_STS_HalfDuplex::writeSCS(unsigned char *nDat, int nLen) {
  if (nDat == nullptr || nLen <= 0) {
    return 0;
  }

  setTx();
  size_t written = _serial->write(nDat, (size_t)nLen);
  _txBytesWritten += (int)written;
  return (int)written;
}

int SMS_STS_HalfDuplex::writeSCS(unsigned char bDat) {
  setTx();
  size_t written = _serial->write(&bDat, 1);
  _txBytesWritten += (int)written;
  return (int)written;
}

int SMS_STS_HalfDuplex::readSCS(unsigned char *nDat, int nLen) {
  setRx();

  int size = 0;
  unsigned long t_begin = millis();
  while (size < nLen) {
    int c = _serial->read();
    if (c != -1) {
      if (nDat != nullptr) {
        nDat[size] = (unsigned char)c;
      }
      size++;
      t_begin = millis();
    }

    if (millis() - t_begin > IOTimeOut) {
      break;
    }
  }

  return size;
}

void SMS_STS_HalfDuplex::rFlushSCS() {
  setRx();
  _txBytesWritten = 0;

  // Drop any stale bytes that may be sitting in the RX FIFO before a new
  // command/response transaction starts.
  while (_serial->read() != -1) {
  }
}

void SMS_STS_HalfDuplex::wFlushSCS() {
  // Wait until the last command byte has left the UART shift register.
  _serial->flush();
  delayMicroseconds(20);

  // Because RX and TX share one GPIO, the UART sees its own transmission as
  // loopback echo. Discard exactly the bytes we transmitted so the next
  // read() starts at the beginning of the servo response.
  int remaining = _txBytesWritten;
  unsigned long t = millis();
  while (remaining > 0) {
    if (_serial->read() != -1) {
      remaining--;
      t = millis();
    }
    if (millis() - t > 2) {
      break;  // safety timeout, should never happen
    }
  }
  _txBytesWritten = 0;

  // Hand the shared line back to the servo before reading its reply.
  setRx();
}
