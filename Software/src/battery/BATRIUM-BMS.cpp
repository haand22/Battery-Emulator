#include "BATRIUM-BMS.h"
#include "../battery/BATTERIES.h"
#include "../communication/can/comm_can.h"
#include "../datalayer/datalayer.h"
#include "../devboard/utils/events.h"

void BatriumBattery::update_values() {

  // Checked
  datalayer.battery.status.real_soc = state_of_charge;
  datalayer.battery.status.soh_pptt = state_of_health;
  datalayer.battery.status.voltage_dV = shunt_voltage_mV / 100;
  datalayer.battery.status.current_dA = shunt_current_mA / 100;
  datalayer.battery.status.max_charge_power_W = (charge_target_current_mA * shunt_voltage_mV) / 1000000L;

  datalayer.battery.status.max_discharge_power_W = (discharge_target_current_mA * shunt_voltage_mV) / 1000000L;

  datalayer.battery.status.remaining_capacity_Wh = (remaining_capacity_mAh * shunt_voltage_mV) / 1000000L;

  datalayer.battery.status.temperature_min_dC = cell_temp_min_degC * 10;
  datalayer.battery.status.temperature_max_dC = cell_temp_max_degC * 10;

  //Map all cell voltages to the global array
  for (uint8_t i = 0; i < MAX_AMOUNT_CELLS; i++) {
    datalayer.battery.status.cell_voltages_mV[i] = cell_voltage_avg_mV;  //Default to average
  }

  datalayer.battery.status.cell_max_voltage_mV = cell_voltage_max_mV;
  datalayer.battery.status.cell_min_voltage_mV = cell_voltage_min_mV;

  // Calculate number of cells based on shunt voltage and average cell voltage
  uint16_t amount_of_detected_cells = 0;
  if (cell_voltage_avg_mV > 0) {
    amount_of_detected_cells = (shunt_voltage_mV + cell_voltage_avg_mV / 2) / cell_voltage_avg_mV;
  }
  if (amount_of_detected_cells < MAX_AMOUNT_CELLS) {
    datalayer.battery.info.number_of_cells = amount_of_detected_cells;
  }

  datalayer.battery.status.balancing_status =
      cell_balancing_flags > 0 ? BALANCING_STATUS_ACTIVE : BALANCING_STATUS_READY;
}

void BatriumBattery::handle_incoming_can_frame(CAN_frame rx_frame) {
  uint32_t id = rx_frame.ID;

  switch (id) {
    // =========================================================
    // Device versioning (Base + 0x00)
    // =========================================================
    case BATRIUM_BASE_ADDR + 0x00:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      hardware_version = (uint16_t)(rx_frame.data.u8[1] << 8 | rx_frame.data.u8[0]);

      firmware_version = (uint16_t)(rx_frame.data.u8[3] << 8 | rx_frame.data.u8[2]);

      device_serial_number = (uint32_t)((rx_frame.data.u8[7] << 24) | (rx_frame.data.u8[6] << 16) |
                                        (rx_frame.data.u8[5] << 8) | rx_frame.data.u8[4]);
      break;

    // =========================================================
    // Cell voltage limits (Base + 0x01)
    // =========================================================
    case BATRIUM_BASE_ADDR + 0x01:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      cell_voltage_min_mV = (uint16_t)(rx_frame.data.u8[1] << 8 | rx_frame.data.u8[0]);

      cell_voltage_max_mV = (uint16_t)(rx_frame.data.u8[3] << 8 | rx_frame.data.u8[2]);

      cell_voltage_avg_mV = (uint16_t)(rx_frame.data.u8[5] << 8 | rx_frame.data.u8[4]);
      cell_voltage_min_index = rx_frame.data.u8[6];
      cell_voltage_max_index = rx_frame.data.u8[7];
      break;

    // =========================================================
    // Cell temperature limits (Base + 0x02)
    // =========================================================
    case BATRIUM_BASE_ADDR + 0x02:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      cell_temp_min_degC = (int8_t)rx_frame.data.u8[0] - (int8_t)40;  // raw − 40 = °C
      cell_temp_max_degC = (int8_t)rx_frame.data.u8[1] - (int8_t)40;  // raw − 40 = °C
      cell_temp_avg_degC = (int8_t)rx_frame.data.u8[2] - (int8_t)40;  // raw − 40 = °C
      cell_temp_min_index = rx_frame.data.u8[3];
      cell_temp_max_index = rx_frame.data.u8[4];
      break;

    // =========================================================
    // Cell bypass summary (Base + 0x03)
    // =========================================================
    case BATRIUM_BASE_ADDR + 0x03:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      cells_in_bypass = rx_frame.data.u8[0];
      cells_initial_bypass = rx_frame.data.u8[1];
      cells_final_bypass = rx_frame.data.u8[2];
      break;

    // =========================================================
    // Shunt power monitoring (Base + 0x04)
    // =========================================================
    case BATRIUM_BASE_ADDR + 0x04:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      shunt_voltage_mV = (int32_t)(rx_frame.data.u8[1] << 8 | rx_frame.data.u8[0]) * (int32_t)100;  // Convert to mV

      shunt_current_mA = (int32_t)(rx_frame.data.u8[3] << 8 | rx_frame.data.u8[2]) * (int32_t)100;  // Convert to mA

      shunt_power_mW = (int32_t)(rx_frame.data.u8[5] << 8 | rx_frame.data.u8[4]) * 10;  // Convert to mW
      break;

    // =========================================================
    // Shunt state monitoring (Base + 0x05)
    // =========================================================
    case BATRIUM_BASE_ADDR + 0x05:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      state_of_charge = (int16_t)(rx_frame.data.u8[1] << 8 | rx_frame.data.u8[0]);  // 0.01 %/bit

      state_of_health = (int16_t)(rx_frame.data.u8[3] << 8 | rx_frame.data.u8[2]);  // 0.01 %/bit

      remaining_capacity_mAh =
          abs((int16_t)(rx_frame.data.u8[5] << 8 | rx_frame.data.u8[4]) * (uint16_t)100);  // -0.1 Ah / bit

      nominal_capacity_mAh = (uint16_t)(rx_frame.data.u8[7] << 8 | rx_frame.data.u8[6]) * (uint16_t)100;  // 100 mAh/bit
      break;

    // =========================================================
    // Remote control target limits (Base + 0x06)
    // =========================================================
    case BATRIUM_BASE_ADDR + 0x06:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      charge_target_voltage_mV =
          (uint16_t)(rx_frame.data.u8[1] << 8 | rx_frame.data.u8[0]) * (uint16_t)100;  // Convert to mV

      charge_target_current_mA =
          (uint16_t)(rx_frame.data.u8[3] << 8 | rx_frame.data.u8[2]) * (uint16_t)100;  // Convert to mA

      discharge_target_voltage_mV =
          (uint16_t)(rx_frame.data.u8[5] << 8 | rx_frame.data.u8[4]) * (uint16_t)100;  // Convert to mV
      discharge_target_current_mA =
          (uint16_t)(rx_frame.data.u8[7] << 8 | rx_frame.data.u8[6]) * (uint16_t)100;  // Convert to mA
      break;

    // =========================================================
    // Control flag logic state (Base + 0x07)
    // =========================================================
    case BATRIUM_BASE_ADDR + 0x07:
      datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      critical_control_flags = rx_frame.data.u8[0];
      charge_control_flags = rx_frame.data.u8[1];
      discharge_control_flags = rx_frame.data.u8[2];
      heat_control_flags = rx_frame.data.u8[3];
      cool_control_flags = rx_frame.data.u8[4];
      cell_balancing_flags = rx_frame.data.u8[5];
      break;

    default:
      // Ignorer other CAN IDs
      break;
  }
}

void BatriumBattery::transmit_can(unsigned long currentMillis) {
  // No transmission needed for this integration
}

void BatriumBattery::setup(void) {  // Performs one time setup at startup
  strncpy(datalayer.system.info.battery_protocol, Name, 63);
  datalayer.system.info.battery_protocol[63] = '\0';
  datalayer.battery.info.max_design_voltage_dV = user_selected_max_pack_voltage_dV;
  datalayer.battery.info.min_design_voltage_dV = user_selected_min_pack_voltage_dV;
  datalayer.battery.info.max_cell_voltage_mV = user_selected_max_cell_voltage_mV;
  datalayer.battery.info.min_cell_voltage_mV = user_selected_min_cell_voltage_mV;
  datalayer.system.status.battery_allows_contactor_closing = true;
}
