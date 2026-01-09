#ifndef BATRIUM_BMS_H
#define BATRIUM_BMS_H

#define BATRIUM_BASE_ADDR 0x300

#include "../system_settings.h"
#include "CanBattery.h"

class BatriumBattery : public CanBattery {
 public:
  virtual void setup(void);
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual void update_values();
  virtual void transmit_can(unsigned long currentMillis);
  static constexpr const char* Name = "DIY battery with Batrium BMS";

 private:
  // ===== Device versioning (ID + 0x00) =====
  uint16_t hardware_version = 0;      // Raw HW version, uint16, range 0–65000 (vendor-defined)
  uint16_t firmware_version = 0;      // Raw FW version, uint16, e.g. 4.0 encoded as vendor-defined
  uint32_t device_serial_number = 0;  // Device serial number, uint32, unique identifier

  // ===== Cell voltage limits (ID + 0x01) =====
  uint16_t cell_voltage_min_mV = 0;    // Min cell voltage, 1 mV/bit, range 0–65000 mV
  uint16_t cell_voltage_max_mV = 0;    // Max cell voltage, 1 mV/bit, range 0–65000 mV
  uint16_t cell_voltage_avg_mV = 0;    // Avg cell voltage, 1 mV/bit, range 0–65000 mV
  uint8_t cell_voltage_min_index = 0;  // Cell index of min voltage, 0–250
  uint8_t cell_voltage_max_index = 0;  // Cell index of max voltage, 0–250

  // ===== Cell temperature limits (ID + 0x02) =====
  int8_t cell_temp_min_degC = 0;    // Min cell temp, 1 °C/bit, offset +40 → T = raw − 40 (−40…+125 °C)
  int8_t cell_temp_max_degC = 0;    // Max cell temp, 1 °C/bit, offset +40 → T = raw − 40 (−40…+125 °C)
  int8_t cell_temp_avg_degC = 0;    // Avg cell temp, 1 °C/bit, offset +40 → T = raw − 40
  uint8_t cell_temp_min_index = 0;  // Cell index of min temperature, 0–250
  uint8_t cell_temp_max_index = 0;  // Cell index of max temperature, 0–250

  // ===== Cell bypass summary (ID + 0x03) =====
  uint8_t cells_in_bypass = 0;       // Number of cells in bypass, count
  uint8_t cells_initial_bypass = 0;  // Number of cells entering bypass, count
  uint8_t cells_final_bypass = 0;    // Number of cells leaving bypass, count

  // ===== Shunt power monitoring (ID + 0x04) =====
  int32_t shunt_voltage_mV = 0;  // Shunt voltage, 100 mV/bit, signed, range 0–400.0 V
  int32_t shunt_current_mA = 0;  // Shunt current, 100 mA/bit, signed (+charge / −discharge)
  int32_t shunt_power_mW = 0;    // Shunt power, 10 mW/bit, signed (+charge / −discharge)

  // ===== Shunt state monitoring (ID + 0x05) =====
  int16_t state_of_charge = 0;          // SoC [%*100], 0.01 %/bit, signed, range −10.00…110.00 %
  int16_t state_of_health = 0;          // SoH [%*100], 0.01 %/bit, signed
  uint16_t remaining_capacity_mAh = 0;  // Remaining capacity, 10 mAh/bit → Ah = raw × 0.01
  uint16_t nominal_capacity_mAh = 0;    // Nominal capacity, 10 mAh/bit → Ah = raw × 0.01

  // ===== Remote control target limits (ID + 0x06) =====
  uint16_t charge_target_voltage_mV = 0;     // Charge target voltage, scale16 (e.g. 10 mV/bit → 5400 = 54.00 V)
  uint16_t charge_target_current_mA = 0;     // Charge target current, scale16 (e.g. 100 mA/bit → 1200 = 120.0 A)
  uint16_t discharge_target_voltage_mV = 0;  // Discharge target voltage, scale16 (user-defined)
  uint16_t discharge_target_current_mA = 0;  // Discharge target current, scale16 (e.g. 100 mA/bit)

  // ===== Control flag logic state (ID + 0x07) =====
  uint8_t critical_control_flags = 0;   // Bit0=OK state, Bit1=Transition, Bit2=Precharge
  uint8_t charge_control_flags = 0;     // Bit0=On, Bit1=Transition, Bit2=Limited power
  uint8_t discharge_control_flags = 0;  // Bit0=On, Bit1=Transition, Bit2=Limited power
  uint8_t heat_control_flags = 0;       // Bit0=On, Bit1=Transition
  uint8_t cool_control_flags = 0;       // Bit0=On, Bit1=Transition
  uint8_t cell_balancing_flags = 0;     // Bit0=Cells in bypass, Bit1=Bypass temp relief
};

#endif
