#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Frsky_Telem/AP_Frsky_config.h>

#ifndef AP_RCPROTOCOL_ENABLED
#define AP_RCPROTOCOL_ENABLED 1
#endif

#ifndef AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#define AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED AP_RCPROTOCOL_ENABLED
#endif

#ifndef AP_RCPROTOCOL_CRSF_ENABLED
#define AP_RCPROTOCOL_CRSF_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RCPROTOCOL_DRONECAN_ENABLED
#define AP_RCPROTOCOL_DRONECAN_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED && HAL_ENABLE_DRONECAN_DRIVERS
#endif

#ifndef AP_RCPROTOCOL_FPORT_ENABLED
#define AP_RCPROTOCOL_FPORT_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif
#ifndef AP_RCPROTOCOL_FPORT2_ENABLED
#define AP_RCPROTOCOL_FPORT2_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED && AP_FRSKY_SPORT_TELEM_ENABLED
#endif

#ifndef AP_RCPROTOCOL_IBUS_ENABLED
#define AP_RCPROTOCOL_IBUS_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RCPROTOCOL_PPMSUM_ENABLED
#define AP_RCPROTOCOL_PPMSUM_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RCPROTOCOL_SBUS_ENABLED
#define AP_RCPROTOCOL_SBUS_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RCPROTOCOL_SRXL_ENABLED
#define AP_RCPROTOCOL_SRXL_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RCPROTOCOL_SRXL2_ENABLED
#define AP_RCPROTOCOL_SRXL2_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RCPROTOCOL_ST24_ENABLED
#define AP_RCPROTOCOL_ST24_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RCPROTOCOL_SUMD_ENABLED
#define AP_RCPROTOCOL_SUMD_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RCPROTOCOL_SBUS_NI_ENABLED
#define AP_RCPROTOCOL_SBUS_NI_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED && AP_RCPROTOCOL_SBUS_ENABLED
#endif

#ifndef AP_RCPROTOCOL_FASTSBUS_ENABLED
#define AP_RCPROTOCOL_FASTSBUS_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED && AP_RCPROTOCOL_SBUS_ENABLED
#endif

#ifndef AP_RCPROTOCOL_GHST_ENABLED
#define AP_RCPROTOCOL_GHST_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif
