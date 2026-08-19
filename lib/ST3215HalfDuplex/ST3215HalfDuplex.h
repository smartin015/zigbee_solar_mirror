#pragma once

#include <Arduino.h>
#include <SMS_STS.h>
#include <driver/gpio.h>

// Half-duplex single-wire transport for Waveshare ST-series bus servos
// (ST3215, STS3215, etc.).
//
// A hardware UART is used for accurate 1 Mbps timing, with both UART RX and
// TX mapped to the same GPIO (the servo DATA line). The TX pad driver is
// tri-stated while receiving, so the servo can drive the shared line without
// bus contention.
class SMS_STS_HalfDuplex : public SMS_STS {
public:
  SMS_STS_HalfDuplex();

  // pin:    Arduino pin number connected to servo DATA (D8 == GPIO19 on the
  //         Seeed Studio XIAO ESP32C6).
  // serial: hardware UART to use (default Serial1).
  void begin(int8_t pin, HardwareSerial &serial = Serial1);

  // Permanently reassign a servo's bus ID (stored in the servo EEPROM).
  // Only one servo may be connected to the DATA line while changing its ID.
  //
  // Return values:
  //   1  success
  //  -1  invalid new ID (0xFE is the broadcast address, 0xFF reserved)
  //  -2  unlock failed (no ack and the new ID is not reachable)
  //  -3  new ID not reachable after writing (ID did not change)
  //  -4  ID changed, but the EEPROM write protection could not be re-locked
  int changeId(uint8_t currentId, uint8_t newId);

protected:
  int writeSCS(unsigned char *nDat, int nLen) override;
  int writeSCS(unsigned char bDat) override;
  int readSCS(unsigned char *nDat, int nLen) override;
  void rFlushSCS() override;
  void wFlushSCS() override;

private:
  void setTx();
  void setRx();

  HardwareSerial *_serial = nullptr;
  int8_t _pin = -1;
  uint32_t _mask = 0;
  int _txBytesWritten = 0;
  bool _txMode = false;
};
