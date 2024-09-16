#pragma once

typedef enum {
    SIM_CARRIER_UNKNOWN,
    SIM_CARRIER_SEARCHING,
    SIM_CARRIER_VZW,
    SIM_CARRIER_ATT,
    SIM_CARRIER_TMO,
} SIM_Carrier_e;

typedef enum {
    SIM_GNSS_NO_FIX,
    SIM_GNSS_FIX,
} SIM_GnssFixStatus_e;

void SIM_StartDevice(void);