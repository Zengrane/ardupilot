/* 
Soaring Controller class by Samuel Tabor
Provides a layer between the thermal centring algorithm and the main code for managing navigation targets, data logging, tuning parameters, algorithm inputs and eventually other soaring strategies such as speed-to-fly. AP_TECS libary used for reference.
*/
#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <DataFlash/DataFlash.h>
#include <AP_Math/AP_Math.h>
#include "ExtendedKalmanFilter.h"
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>

#define EXPECTED_THERMALLING_SINK 0.7
#define INITIAL_THERMAL_STRENGTH 2.0
#define INITIAL_THERMAL_RADIUS 30.0 //150.0
#define INITIAL_STRENGTH_COVARIANCE 0.0049
#define INITIAL_RADIUS_COVARIANCE 2500.0
#define INITIAL_POSITION_COVARIANCE 300.0
#define ASPD_FILT 0.05
#define TE_FILT 0.03
#define TE_FILT_DISPLAYED 0.15

class SoaringController
{
    ExtendedKalmanFilter _ekf;
    AP_AHRS &_ahrs;
    AP_SpdHgtControl *&_spdHgt;
    const AP_Vehicle::FixedWing &_aparm;

    // store aircraft location at last update
    struct Location _prev_update_location;

    // store aircraft location at last update
    struct Location _prev_vario_update_location;

    // store time thermal was entered for hysteresis
    unsigned long _thermal_start_time_us;

    // store time cruise was entered for hysteresis
    unsigned long _cruise_start_time_us;

    // store time of last update
    unsigned long _prev_update_time;

    // store time of last update
    unsigned long _prev_vario_update_time;

    float _vario_reading;
    float _filtered_vario_reading;
    float _last_alt;
    float _alt;
    float _last_aspd;
    float _last_roll;
    float _last_total_E;
    bool _new_data;
    float _loiter_rad;
    bool _throttle_suppressed;
    
    float _aspd_filt;
    uint8_t _msgid;
    uint8_t _msgid2;
    float correct_netto_rate(float climb_rate, float phi, float aspd);
    float McCready(float alt);
    void get_wind_corrected_drift(const Location *current_loc, const Vector3f *wind, float *wind_drift_x, float *wind_drift_y, float *dx, float *dy);
    void get_altitude_wrt_home(float *alt);
  
protected:
    AP_Int8 soar_active;
    AP_Float thermal_vspeed;
    AP_Float thermal_q1;
    AP_Float thermal_q2;
    AP_Float thermal_r;
    AP_Float thermal_distance_ahead;
    AP_Int16 min_thermal_s;
    AP_Int16 min_cruise_s;
    AP_Float polar_CD0;
    AP_Float polar_B;
    AP_Float polar_K;
    AP_Float alt_max;
    AP_Float alt_min;
    AP_Float alt_cutoff;
        
public:
    SoaringController(AP_AHRS &ahrs, AP_SpdHgtControl *&spdHgt, const AP_Vehicle::FixedWing &parms, uint8_t msgid, uint8_t msgid2) :
        _ahrs(ahrs),
        _spdHgt(spdHgt),
        _aparm(parms),
        _thermal_start_time_us(0),
        _cruise_start_time_us(0),
        _prev_update_time(0),
        _prev_vario_update_time(0),
        _vario_reading(0.0f),
        _filtered_vario_reading(0.0f),
        _last_alt(0.0f),
        _alt(0.0f),
        _last_aspd(0.0f),
        _last_roll(0.0f),
        _last_total_E(0.0f),
        _new_data(false),
        _loiter_rad(parms.loiter_radius),
        _throttle_suppressed(true),
        _aspd_filt(0.0f),
        _msgid(msgid),
        _msgid2(msgid2)
    {
        AP_Param::setup_object_defaults(this, var_info);
        ahrs.get_position(_prev_update_location);
        ahrs.get_position(_prev_vario_update_location);  
    }
    
    float _displayed_vario_reading;  
    // this supports the TECS_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];
    void get_target(Location & wp);
    bool suppress_throttle();
    void log_data();
    bool check_thermal_criteria();
    bool check_cruise_criteria();
    bool check_init_thermal_criteria();
    void init_thermalling();
    void init_cruising();
    void update_thermalling();
    void update_cruising();
    bool is_active();
    bool get_throttle_suppressed() { return _throttle_suppressed; }
    void set_throttle_suppressed(bool suppressed) { _throttle_suppressed = suppressed; }
    
    // Soaring log structure
    struct PACKED log_Thermal_Tuning {
        LOG_PACKET_HEADER;      
        uint64_t time_us;
        float netto_rate;
        float dx;
        float dy;
        float x0;
        float x1;       
        float x2;       
        float x3;       
        uint32_t lat;       
        uint32_t lng;       
        float alt;
        float dx_w;
        float dy_w;
    } log_tuning;
    
    struct PACKED log_Vario_Tuning {
        LOG_PACKET_HEADER;      
        uint64_t time_us;
        float aspd_raw;
        float aspd_filt;
        float alt;
        float roll;
        float raw;
        float filt;
        float wind_x;
        float wind_y;
        float dx;
        float dy;
    } log_vario_tuning;
 
    void update_vario();
};