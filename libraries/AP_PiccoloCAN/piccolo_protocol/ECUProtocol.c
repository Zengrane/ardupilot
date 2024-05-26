// ECUProtocol.c was generated by ProtoGen version 3.2.a

/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */
 

#include "ECUProtocol.h"

/*!
 * \brief Lookup label for 'ECUPackets' enum entry
 * 
 * \param value is the integer value of the enum entry
 * \return string label of the given entry
 */
const char* ECUPackets_EnumLabel(int value)
{
    switch (value)
    {
    default:
        return "";
    case PKT_ECU_TELEMETRY_FAST:
        return translateECU("PKT_ECU_TELEMETRY_FAST");
    case PKT_ECU_TELEMETRY_SLOW_0:
        return translateECU("PKT_ECU_TELEMETRY_SLOW_0");
    case PKT_ECU_TELEMETRY_SLOW_1:
        return translateECU("PKT_ECU_TELEMETRY_SLOW_1");
    case PKT_ECU_TELEMETRY_SLOW_2:
        return translateECU("PKT_ECU_TELEMETRY_SLOW_2");
    case PKT_ECU_TELEMETRY_SLOW_3:
        return translateECU("PKT_ECU_TELEMETRY_SLOW_3");
    case PKT_ECU_THROTTLE_CALIBRATION:
        return translateECU("PKT_ECU_THROTTLE_CALIBRATION");
    case PKT_ECU_THROTTLE:
        return translateECU("PKT_ECU_THROTTLE");
    case PKT_ECU_RPM_COMMAND:
        return translateECU("PKT_ECU_RPM_COMMAND");
    case PKT_ECU_RPM_CALIBRATION:
        return translateECU("PKT_ECU_RPM_CALIBRATION");
    case PKT_ECU_HARDWARE_CONFIG:
        return translateECU("PKT_ECU_HARDWARE_CONFIG");
    case PKT_ECU_SOFTWARE_VERSION:
        return translateECU("PKT_ECU_SOFTWARE_VERSION");
    case PKT_ECU_TPS_DELAY_CONFIG:
        return translateECU("PKT_ECU_TPS_DELAY_CONFIG");
    case PKT_ECU_TELEMETRY_SETTINGS:
        return translateECU("PKT_ECU_TELEMETRY_SETTINGS");
    case PKT_ECU_PUMP_CONFIG:
        return translateECU("PKT_ECU_PUMP_CONFIG");
    case PKT_ECU_ERROR_MSG:
        return translateECU("PKT_ECU_ERROR_MSG");
    case PKT_ECU_POWER_CYCLES:
        return translateECU("PKT_ECU_POWER_CYCLES");
    case PKT_ECU_PUMP_2_CONFIG:
        return translateECU("PKT_ECU_PUMP_2_CONFIG");
    case PKT_ECU_PUMP_DEBUG:
        return translateECU("PKT_ECU_PUMP_DEBUG");
    case PKT_ECU_TOTAL_ENGINE_TIME:
        return translateECU("PKT_ECU_TOTAL_ENGINE_TIME");
    case PKT_ECU_SYS_CMD:
        return translateECU("PKT_ECU_SYS_CMD");
    case PKT_ECU_USER_DATA:
        return translateECU("PKT_ECU_USER_DATA");
    case PKT_ECU_THROTTLE_CURVE_0:
        return translateECU("PKT_ECU_THROTTLE_CURVE_0");
    case PKT_ECU_THROTTLE_CURVE_1:
        return translateECU("PKT_ECU_THROTTLE_CURVE_1");
    case PKT_ECU_GPIO:
        return translateECU("PKT_ECU_GPIO");
    case PKT_ECU_SETTINGS_DATA:
        return translateECU("PKT_ECU_SETTINGS_DATA");
    case PKT_ECU_CHT_LOOP:
        return translateECU("PKT_ECU_CHT_LOOP");
    }
}


/*!
 * \brief Lookup label for 'ECUSystemCommands' enum entry
 * 
 * \param value is the integer value of the enum entry
 * \return string label of the given entry
 */
const char* ECUSystemCommands_EnumLabel(int value)
{
    switch (value)
    {
    default:
        return "";
    case CMD_ECU_CALIBRATE_ANALOG_CLOSED:
        return translateECU("CMD_ECU_CALIBRATE_ANALOG_CLOSED");
    case CMD_ECU_CALIBRATE_ANALOG_OPEN:
        return translateECU("CMD_ECU_CALIBRATE_ANALOG_OPEN");
    case CMD_ECU_CALIBRATE_PULSE_CLOSED:
        return translateECU("CMD_ECU_CALIBRATE_PULSE_CLOSED");
    case CMD_ECU_CALIBRATE_PULSE_OPEN:
        return translateECU("CMD_ECU_CALIBRATE_PULSE_OPEN");
    case CMD_ECU_CALIBRATE_PULSE_WRITE:
        return translateECU("CMD_ECU_CALIBRATE_PULSE_WRITE");
    case CMD_ECU_SET_OUTPUT_DRIVER:
        return translateECU("CMD_ECU_SET_OUTPUT_DRIVER");
    case CMD_ECU_SET_THROTTLE_CURVE_ACTIVE:
        return translateECU("CMD_ECU_SET_THROTTLE_CURVE_ACTIVE");
    case CMD_ECU_SET_THROTTLE_CURVE_ELEMENT:
        return translateECU("CMD_ECU_SET_THROTTLE_CURVE_ELEMENT");
    case CMD_ECU_REQUEST_THROTTLE_CURVE_DATA:
        return translateECU("CMD_ECU_REQUEST_THROTTLE_CURVE_DATA");
    case CMD_ECU_RESET_FUEL_USED:
        return translateECU("CMD_ECU_RESET_FUEL_USED");
    case CMD_ECU_SET_FUEL_USED_DIVISOR:
        return translateECU("CMD_ECU_SET_FUEL_USED_DIVISOR");
    case CMD_ECU_FUEL_USED_RESET_ON_STARTUP:
        return translateECU("CMD_ECU_FUEL_USED_RESET_ON_STARTUP");
    case CMD_ECU_SET_GOVERNOR_MODE:
        return translateECU("CMD_ECU_SET_GOVERNOR_MODE");
    case CMD_ECU_SET_SERVO_CAN_MODE:
        return translateECU("CMD_ECU_SET_SERVO_CAN_MODE");
    case CMD_ECU_RESET_INTO_BOOTLOADER:
        return translateECU("CMD_ECU_RESET_INTO_BOOTLOADER");
    case CMD_ECU_RESET_DEFAULT_SETTINGS:
        return translateECU("CMD_ECU_RESET_DEFAULT_SETTINGS");
    case CMD_ECU_SET_SERIAL_MODE:
        return translateECU("CMD_ECU_SET_SERIAL_MODE");
    case CMD_ECU_SET_NODE_ID:
        return translateECU("CMD_ECU_SET_NODE_ID");
    case CMD_ECU_SET_USER_DATA:
        return translateECU("CMD_ECU_SET_USER_DATA");
    case CMD_ECU_RESET_ENGINE_TIME:
        return translateECU("CMD_ECU_RESET_ENGINE_TIME");
    case CMD_ECU_RESET_ECU:
        return translateECU("CMD_ECU_RESET_ECU");
    }
}

// end of ECUProtocol.c
