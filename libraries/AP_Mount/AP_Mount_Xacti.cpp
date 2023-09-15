#include "AP_Mount_Xacti.h"

#if HAL_MOUNT_XACTI_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>

extern const AP_HAL::HAL& hal;

#define LOG_TAG "Mount"
#define XACTI_PARAM_SINGLESHOT "SingleShot"
#define XACTI_PARAM_RECORDING "Recording"
#define XACTI_PARAM_FOCUSMODE "FocusMode"
#define XACTI_PARAM_SENSORMODE "SensorMode"
#define XACTI_PARAM_DIGITALZOOM "DigitalZoomMagnification"
#define XACTI_PARAM_FIRMWAREVERSION "FirmwareVersion"
#define XACTI_PARAM_STATUS "Status"

#define XACTI_MSG_SEND_MIN_MS 20                    // messages should not be sent to camera more often than 20ms
#define XACTI_ZOOM_RATE_UPDATE_INTERVAL_MS  500     // zoom rate control increments zoom by 10% up or down every 0.5sec
#define XACTI_STATUS_REQ_INTERVAL_MS 3000           // request status every 3 seconds

#define AP_MOUNT_XACTI_DEBUG 0
#define debug(fmt, args ...) do { if (AP_MOUNT_XACTI_DEBUG) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Xacti: " fmt, ## args); } } while (0)

bool AP_Mount_Xacti::_subscribed = false;
AP_Mount_Xacti::DetectedModules AP_Mount_Xacti::_detected_modules[];
HAL_Semaphore AP_Mount_Xacti::_sem_registry;
const char* AP_Mount_Xacti::send_text_prefix = "Xacti:";
const char* AP_Mount_Xacti::sensor_mode_str[] = { "RGB", "IR", "PIP", "NDVI" };

// Constructor
AP_Mount_Xacti::AP_Mount_Xacti(class AP_Mount &frontend, class AP_Mount_Params &params, uint8_t instance) :
    AP_Mount_Backend(frontend, params, instance)
{
    register_backend();

    param_int_cb = FUNCTOR_BIND_MEMBER(&AP_Mount_Xacti::handle_param_get_set_response_int, bool, AP_DroneCAN*, const uint8_t, const char*, int32_t &);
    param_string_cb = FUNCTOR_BIND_MEMBER(&AP_Mount_Xacti::handle_param_get_set_response_string, bool, AP_DroneCAN*, const uint8_t, const char*, AP_DroneCAN::string &);
    param_save_cb = FUNCTOR_BIND_MEMBER(&AP_Mount_Xacti::handle_param_save_response, void, AP_DroneCAN*, const uint8_t, bool);
}

// init - performs any required initialisation for this instance
void AP_Mount_Xacti::init()
{
    _initialised = true;
}

// update mount position - should be called periodically
void AP_Mount_Xacti::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // return immediately if any message sent is unlikely to be processed
    if (!is_safe_to_send()) {
        return;
    }

    // get firmware version
    if (request_firmware_version()) {
        return;
    }

    // request status
    if (request_status()) {
        return;
    }

    // periodically send copter attitude and GPS status
    if (send_copter_att_status()) {
        // if message sent avoid sending other messages
        return;
    }

    // update zoom rate control
    if (update_zoom_rate_control()) {
        // if message sent avoid sending other messages
        return;
    }

    // update based on mount mode
    switch (get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &angle_bf_target = _params.retract_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
            break;
        }

        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &angle_bf_target = _params.neutral_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
            break;
        }

        case MAV_MOUNT_MODE_MAVLINK_TARGETING: {
            // mavlink targets are set while handling the incoming message
            break;
        }

        case MAV_MOUNT_MODE_RC_TARGETING: {
            // update targets using pilot's RC inputs
            MountTarget rc_target;
            get_rc_target(mnt_target.target_type, rc_target);
            switch (mnt_target.target_type) {
            case MountTargetType::ANGLE:
                mnt_target.angle_rad = rc_target;
                break;
            case MountTargetType::RATE:
                mnt_target.rate_rads = rc_target;
                break;
            }
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (get_angle_target_to_roi(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        // point mount to Home location
        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        // point mount to another vehicle
        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (get_angle_target_to_sysid(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        default:
            // we do not know this mode so raise internal error
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
    }

    // send target angles or rates depending on the target type
    switch (mnt_target.target_type) {
        case MountTargetType::ANGLE:
            send_target_angles(mnt_target.angle_rad.pitch, mnt_target.angle_rad.yaw, mnt_target.angle_rad.yaw_is_ef);
            break;
        case MountTargetType::RATE:
            send_target_rates(mnt_target.rate_rads.pitch, mnt_target.rate_rads.yaw, mnt_target.rate_rads.yaw_is_ef);
            break;
    }
}

// return true if healthy
bool AP_Mount_Xacti::healthy() const
{
    // unhealthy until gimbal has been found and replied with firmware version and no motor errors
    if (!_initialised || !_firmware_version.received || _motor_error) {
        return false;
    }

    // unhealthy if attitude information NOT received recently
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_current_attitude_quat_ms > 1000) {
        return false;
    }

    // if we get this far return healthy
    return true;
}

// take a picture.  returns true on success
bool AP_Mount_Xacti::take_picture()
{
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return false;
    }

    // set SingleShot parameter
    return set_param_int32(XACTI_PARAM_SINGLESHOT, 0);
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount_Xacti::record_video(bool start_recording)
{
    return set_param_int32(XACTI_PARAM_RECORDING, start_recording ? 1 : 0);
}

// set zoom specified as a rate or percentage
bool AP_Mount_Xacti::set_zoom(ZoomType zoom_type, float zoom_value)
{
    // zoom rate
    if (zoom_type == ZoomType::RATE) {
        if (is_zero(zoom_value)) {
            // stop zooming
            _zoom_rate_control.enabled = false;
        } else {
            // zoom in or out
            _zoom_rate_control.enabled = true;
            _zoom_rate_control.increment = (zoom_value < 0) ? -100 : 100;
        }
        return true;
    }

    // zoom percentage
    if (zoom_type == ZoomType::PCT) {
        // convert zoom percentage (0 ~ 100) to zoom parameter value (100, 200, 300, ... 1000)
        // 0~9pct:100, 10~19pct:200, ... 90~100pct:1000
        uint16_t zoom_param_value = constrain_uint16(uint16_t(zoom_value * 0.1) * 10, 100, 1000);
        return set_param_int32(XACTI_PARAM_DIGITALZOOM, zoom_param_value);
    }

    // unsupported zoom type
    return false;
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Mount_Xacti::set_focus(FocusType focus_type, float focus_value)
{
    // convert focus type and value to parameter value
    uint8_t focus_param_value;
    switch (focus_type) {
    case FocusType::RATE:
    case FocusType::PCT:
        // focus rate and percentage control not supported so simply switch to manual focus
        // FocusMode of 0:Manual Focus
        focus_param_value = 0;
        break;
    case FocusType::AUTO:
        // FocusMode of 1:Single AutoFocus, 2:Continuous AutoFocus
        focus_param_value = 2;
        break;
    default:
        // unsupported forucs mode
        return SetFocusResult::INVALID_PARAMETERS;
    }

    // set FocusMode parameter
    return set_param_int32(XACTI_PARAM_FOCUSMODE, focus_param_value) ? SetFocusResult::ACCEPTED : SetFocusResult::FAILED;
}

// set camera lens as a value from 0 to 5
bool AP_Mount_Xacti::set_lens(uint8_t lens)
{
    // sanity check
    if (lens > (uint8_t)SensorsMode::NDVI) {
        return false;
    }

    return set_param_int32(XACTI_PARAM_SENSORMODE, lens);
}

// send camera information message to GCS
void AP_Mount_Xacti::send_camera_information(mavlink_channel_t chan) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    static const uint8_t vendor_name[32] = "Xacti";
    static uint8_t model_name[32] = "CX-GB100";
    const char cam_definition_uri[140] {};
    const float NaN = nanf("0x4152");

    // capability flags
    const uint32_t flags = CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                           CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                           CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS;

    // send CAMERA_INFORMATION message
    mavlink_msg_camera_information_send(
        chan,
        AP_HAL::millis(),       // time_boot_ms
        vendor_name,            // vendor_name uint8_t[32]
        model_name,             // model_name uint8_t[32]
        _firmware_version.received ? _firmware_version.mav_ver : 0, // firmware version uint32_t
        NaN,                    // focal_length float (mm)
        NaN,                    // sensor_size_h float (mm)
        NaN,                    // sensor_size_v float (mm)
        0,                      // resolution_h uint16_t (pix)
        0,                      // resolution_v uint16_t (pix)
        0,                      // lens_id uint8_t
        flags,                  // flags uint32_t (CAMERA_CAP_FLAGS)
        0,                      // cam_definition_version uint16_t
        cam_definition_uri,     // cam_definition_uri char[140]
        _instance + 1);         // gimbal_device_id uint8_t
}

// send camera settings message to GCS
void AP_Mount_Xacti::send_camera_settings(mavlink_channel_t chan) const
{
    const float NaN = nanf("0x4152");

    // send CAMERA_SETTINGS message
    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),   // time_boot_ms
        _recording_video ? CAMERA_MODE_VIDEO : CAMERA_MODE_IMAGE,   // camera mode (0:image, 1:video, 2:image survey)
        0,                  // zoomLevel float, percentage from 0 to 100, NaN if unknown
        NaN);               // focusLevel float, percentage from 0 to 100, NaN if unknown
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Xacti::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat = _current_attitude_quat;
    return true;
}

// send target pitch and yaw rates to gimbal
// yaw_is_ef should be true if yaw_rads target is an earth frame rate, false if body_frame
void AP_Mount_Xacti::send_target_rates(float pitch_rads, float yaw_rads, bool yaw_is_ef)
{
    // send gimbal rate target to gimbal
    send_gimbal_control(3, degrees(pitch_rads) * 100, degrees(yaw_rads) * 100);
}

// send target pitch and yaw angles to gimbal
// yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
void AP_Mount_Xacti::send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef)
{
    // convert yaw to body frame
    const float yaw_bf_rad = yaw_is_ef ? wrap_PI(yaw_rad - AP::ahrs().yaw) : yaw_rad;

    // send angle target to gimbal
    send_gimbal_control(2, degrees(pitch_rad) * 100, degrees(yaw_bf_rad) * 100);
}

// subscribe to Xacti DroneCAN messages
void AP_Mount_Xacti::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    // return immediately if DroneCAN is unavailable
    if (ap_dronecan == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Xacti: DroneCAN subscribe failed");
        return;
    }

    _subscribed = true;

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_gimbal_attitude_status, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("gimbal_attitude_status_sub");
        _subscribed = false;
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_gnss_status_req, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("gnss_status_req_sub");
        _subscribed = false;
    }
}

// register backend in detected modules array used to map DroneCAN port and node id to backend
void AP_Mount_Xacti::register_backend()
{
    WITH_SEMAPHORE(_sem_registry);

    // add this backend to _detected_modules array
    _detected_modules[_instance].driver = this;

    // return if devid is zero meaning this backend has not yet been associated with a mount
    const uint32_t devid = (uint32_t)_params.dev_id.get();
    if (devid == 0) {
        return;
    }

    // get DroneCan port from device id
    const uint8_t can_driver_index = AP_HAL::Device::devid_get_bus(devid);
    const uint8_t can_num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        AP_DroneCAN *ap_dronecan = AP_DroneCAN::get_dronecan(i);
        if (ap_dronecan != nullptr && ap_dronecan->get_driver_index() == can_driver_index) {
            _detected_modules[_instance].ap_dronecan = ap_dronecan;
        }
    }

    // get node_id from device id
    _detected_modules[_instance].node_id = AP_HAL::Device::devid_get_address(devid);
}

// find backend associated with the given dronecan port and node_id.  also associates backends with zero node ids
// returns pointer to backend on success, nullptr on failure
AP_Mount_Xacti* AP_Mount_Xacti::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id)
{
    WITH_SEMAPHORE(_sem_registry);

    // exit immediately if DroneCAN is unavailable or invalid node id
    if (ap_dronecan == nullptr || node_id == 0) {
        return nullptr;
    }

    // search for backend with matching dronecan port and node id
    for (uint8_t i = 0; i < ARRAY_SIZE(_detected_modules); i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_dronecan == ap_dronecan &&
            _detected_modules[i].node_id == node_id ) {
            return _detected_modules[i].driver;
        }
    }

    // if we got this far, this dronecan port and node id are not associated with any backend
    // associate with first backend with node id of zero
    for (uint8_t i = 0; i < ARRAY_SIZE(_detected_modules); i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].node_id == 0) {
                _detected_modules[i].ap_dronecan = ap_dronecan;
                _detected_modules[i].node_id = node_id;
                const auto dev_id = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                                ap_dronecan->get_driver_index(),
                                                                node_id, 0);
                _detected_modules[i].driver->set_dev_id(dev_id);
                return _detected_modules[i].driver;
        }
    }

    return nullptr;
}

// handle xacti gimbal attitude status message
void AP_Mount_Xacti::handle_gimbal_attitude_status(AP_DroneCAN* ap_dronecan, const CanardRxTransfer& transfer, const com_xacti_GimbalAttitudeStatus &msg)
{
    // fetch the matching backend driver, node id and gimbal id backend instance
    AP_Mount_Xacti* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);
    if (driver == nullptr) {
        return;
    }

    // convert body-frame Euler angles to Quaternion.  Note yaw direction is reversed from normal
    driver->_current_attitude_quat.from_euler(radians(msg.gimbal_roll * 0.01), radians(msg.gimbal_pitch * 0.01), radians(-msg.gimbal_yaw * 0.01));
    driver->_last_current_attitude_quat_ms = AP_HAL::millis();
}

// handle xacti gnss status request message
void AP_Mount_Xacti::handle_gnss_status_req(AP_DroneCAN* ap_dronecan, const CanardRxTransfer& transfer, const com_xacti_GnssStatusReq &msg)
{
    // sanity check dronecan port
    if (ap_dronecan == nullptr) {
        return;
    }

    // get current location
    uint8_t gps_status = 2;
    Location loc;
    if (!AP::ahrs().get_location(loc)) {
        gps_status = 0;
    }

    // get date and time
    uint16_t year, ms;
    uint8_t month, day, hour, min, sec;
    if (!AP::rtc().get_date_and_time_utc(year, month, day, hour, min, sec, ms)) {
        year = month = day = hour = min = sec = 0;
    }

    // send xacti specific gnss status message
    com_xacti_GnssStatus xacti_gnss_status_msg {};
    xacti_gnss_status_msg.gps_status = gps_status;
    xacti_gnss_status_msg.order = msg.requirement;
    xacti_gnss_status_msg.remain_buffer = 1;
    xacti_gnss_status_msg.utc_year = year;
    xacti_gnss_status_msg.utc_month = month + 1;
    xacti_gnss_status_msg.utc_day = day;
    xacti_gnss_status_msg.utc_hour = hour;
    xacti_gnss_status_msg.utc_minute = min;
    xacti_gnss_status_msg.utc_seconds = sec;
    xacti_gnss_status_msg.latitude = loc.lat * 1e-7;
    xacti_gnss_status_msg.longitude = loc.lng * 1e-7;
    xacti_gnss_status_msg.altitude = loc.alt * 1e-2;
    ap_dronecan->xacti_gnss_status.broadcast(xacti_gnss_status_msg);
}

// handle param get/set response
bool AP_Mount_Xacti::handle_param_get_set_response_int(AP_DroneCAN* ap_dronecan, uint8_t node_id, const char* name, int32_t &value)
{
    // display errors
    const char* err_prefix_str = "Xacti: failed to";
    if (strcmp(name, XACTI_PARAM_SINGLESHOT) == 0) {
        if (value < 0) {
            gcs().send_text(MAV_SEVERITY_ERROR, "%s take pic", err_prefix_str);
        }
        return false;
    }
    if (strcmp(name, XACTI_PARAM_RECORDING) == 0) {
        if (value < 0) {
            _recording_video = false;
            gcs().send_text(MAV_SEVERITY_ERROR, "%s record", err_prefix_str);
        } else {
            _recording_video = (value == 1);
            gcs().send_text(MAV_SEVERITY_INFO, "Xacti: recording %s", _recording_video ? "ON" : "OFF");
        }
        return false;
    }
    if (strcmp(name, XACTI_PARAM_FOCUSMODE) == 0) {
        if (value < 0) {
            gcs().send_text(MAV_SEVERITY_ERROR, "%s change focus", err_prefix_str);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Xacti: %s focus", value == 0 ? "manual" : "auto");
        }
        return false;
    }
    if (strcmp(name, XACTI_PARAM_SENSORMODE) == 0) {
        if (value < 0) {
            gcs().send_text(MAV_SEVERITY_ERROR, "%s change lens", err_prefix_str);
        } else if ((uint32_t)value < ARRAY_SIZE(sensor_mode_str)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Xacti: %s", sensor_mode_str[(uint8_t)value]);
        }
        return false;
    }
    if (strcmp(name, XACTI_PARAM_DIGITALZOOM) == 0) {
        if (value < 0) {
            gcs().send_text(MAV_SEVERITY_ERROR, "%s change zoom", err_prefix_str);
            // disable zoom rate control (if active) to avoid repeated failures
            _zoom_rate_control.enabled = false;
        } else if (value >= 100 && value <= 1000) {
            _last_zoom_param_value = value;
        }
        return false;
    }
    // unhandled parameter get or set
    gcs().send_text(MAV_SEVERITY_INFO, "Xacti: get/set %s res:%ld", name, (long int)value);
    return false;
}

// handle param get/set response
bool AP_Mount_Xacti::handle_param_get_set_response_string(AP_DroneCAN* ap_dronecan, uint8_t node_id, const char* name, AP_DroneCAN::string &value)
{
    if (strcmp(name, XACTI_PARAM_FIRMWAREVERSION) == 0) {
        _firmware_version.received = true;
        const uint8_t len = MIN(value.len, ARRAY_SIZE(_firmware_version.str)-1);
        memcpy(_firmware_version.str, (const char*)value.data, len);
        gcs().send_text(MAV_SEVERITY_INFO, "Mount: Xacti fw:%s", _firmware_version.str);

        // firmware str from gimbal is of the format YYMMDD[b]xx.  Convert to uint32 for reporting to GCS
        if (len >= 9) {
            const char major_str[3] = {_firmware_version.str[0], _firmware_version.str[1], 0};
            const char minor_str[3] = {_firmware_version.str[2], _firmware_version.str[3], 0};
            const char patch_str[3] = {_firmware_version.str[4], _firmware_version.str[5], 0};
            const char dev_str[3] = {_firmware_version.str[7], _firmware_version.str[8], 0};
            const uint8_t major_ver_num = atoi(major_str) & 0xFF;
            const uint8_t minor_ver_num = atoi(minor_str) & 0xFF;
            const uint8_t patch_ver_num = atoi(patch_str) & 0xFF;
            const uint8_t dev_ver_num = atoi(dev_str) & 0xFF;
            _firmware_version.mav_ver = UINT32_VALUE(dev_ver_num, patch_ver_num, minor_ver_num, major_ver_num);
        }
        return false;
    } else if (strcmp(name, XACTI_PARAM_STATUS) == 0) {
        // check for expected length
        const char* error_str = "error";
        if (value.len != sizeof(_status)) {
            gcs().send_text(MAV_SEVERITY_ERROR, "%s status len %s", send_text_prefix, error_str);
            return false;
        }

        // backup error status and copy to structure
        const uint32_t last_error_status = _status.error_status;
        memcpy(&_status, value.data, value.len);

        // report change in status
        uint32_t changed_bits = last_error_status ^ _status.error_status;
        const char* ok_str = "OK";
        if (changed_bits & (uint32_t)ErrorStatus::CANNOT_TAKE_PIC) {
            gcs().send_text(MAV_SEVERITY_INFO, "%s %s take pic", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::CANNOT_TAKE_PIC ? "cannot" : "can");
        }
        if (changed_bits & (uint32_t)ErrorStatus::TIME_NOT_SET) {
            gcs().send_text(MAV_SEVERITY_INFO, "%s time %sset", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::TIME_NOT_SET ? "not " : " ");
        }
        if (changed_bits & (uint32_t)ErrorStatus::MEDIA_ERROR) {
            gcs().send_text(MAV_SEVERITY_INFO, "%s media %s", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::MEDIA_ERROR ? error_str : ok_str);
        }
        if (changed_bits & (uint32_t)ErrorStatus::LENS_ERROR) {
            gcs().send_text(MAV_SEVERITY_INFO, "%s lens %s", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::LENS_ERROR ? error_str : ok_str);
        }
        if (changed_bits & (uint32_t)ErrorStatus::MOTOR_INIT_ERROR) {
            gcs().send_text(MAV_SEVERITY_INFO, "%s motor %s", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::MOTOR_INIT_ERROR ? "init error" : ok_str);
        }
        if (changed_bits & (uint32_t)ErrorStatus::MOTOR_OPERATION_ERROR) {
            gcs().send_text(MAV_SEVERITY_INFO, "%s motor op %s", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::MOTOR_OPERATION_ERROR ? error_str : ok_str);
        }
        if (changed_bits & (uint32_t)ErrorStatus::GIMBAL_CONTROL_ERROR) {
            gcs().send_text(MAV_SEVERITY_INFO, "%s control %s", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::GIMBAL_CONTROL_ERROR ? error_str : ok_str);
        }
        if (changed_bits & (uint32_t)ErrorStatus::TEMP_WARNING) {
            gcs().send_text(MAV_SEVERITY_INFO, "%s temp %s", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::TEMP_WARNING ? "warning" : ok_str);
        }

        // set motor error for health reporting
        _motor_error = _status.error_status & ((uint32_t)ErrorStatus::MOTOR_INIT_ERROR | (uint32_t)ErrorStatus::MOTOR_OPERATION_ERROR | (uint32_t)ErrorStatus::GIMBAL_CONTROL_ERROR);
        return false;
    }

    // unhandled parameter get or set
    gcs().send_text(MAV_SEVERITY_INFO, "%s get/set string %s res:%s", send_text_prefix, name, (const char*)value.data);
    return false;
}

void AP_Mount_Xacti::handle_param_save_response(AP_DroneCAN* ap_dronecan, const uint8_t node_id, bool success)
{
    // display failure to save parameter
    if (!success) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Xacti: CAM%u failed to set param", (int)_instance+1);
    }
}

// helper function to set integer parameters
bool AP_Mount_Xacti::set_param_int32(const char* param_name, int32_t param_value)
{
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return false;
    }

    if (_detected_modules[_instance].ap_dronecan->set_parameter_on_node(_detected_modules[_instance].node_id, param_name, param_value, &param_int_cb)) {
        last_send_getset_param_ms = AP_HAL::millis();
        return true;
    }
    return false;
}

// helper function to get string parameters
bool AP_Mount_Xacti::get_param_string(const char* param_name)
{
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return false;
    }

    if (_detected_modules[_instance].ap_dronecan->get_parameter_on_node(_detected_modules[_instance].node_id, param_name, &param_string_cb)) {
        last_send_getset_param_ms = AP_HAL::millis();
        return true;
    }
    return false;
}

// send gimbal control message via DroneCAN
// mode is 2:angle control or 3:rate control
// pitch_cd is pitch angle in centi-degrees or pitch rate in cds
// yaw_cd is angle in centi-degrees or yaw rate in cds
void AP_Mount_Xacti::send_gimbal_control(uint8_t mode, int16_t pitch_cd, int16_t yaw_cd)
{
    // exit immediately if no DroneCAN port
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return;
    }

    // send at no faster than 5hz
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_gimbal_control_ms < 200) {
        return;
    }
    last_send_gimbal_control_ms = now_ms;

    // send xacti specific gimbal control message
    com_xacti_GimbalControlData gimbal_control_data_msg {};
    gimbal_control_data_msg.pitch_cmd_type = mode;
    gimbal_control_data_msg.yaw_cmd_type = mode;
    gimbal_control_data_msg.pitch_cmd_value = pitch_cd;
    gimbal_control_data_msg.yaw_cmd_value = -yaw_cd;
    _detected_modules[_instance].ap_dronecan->xacti_gimbal_control_data.broadcast(gimbal_control_data_msg);
}

// send copter attitude status message to gimbal
// returns true if sent so that we avoid immediately trying to also send other messages
bool AP_Mount_Xacti::send_copter_att_status()
{
    // exit immediately if no DroneCAN port
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return false;
    }

    // send at no faster than 5hz
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_copter_att_status_ms < 100) {
        return false;
    }

    // send xacti specific vehicle attitude message
    Quaternion veh_att;
    if (!AP::ahrs().get_quaternion(veh_att)) {
        return false;
    }

    last_send_copter_att_status_ms = now_ms;
    com_xacti_CopterAttStatus copter_att_status_msg {};
    copter_att_status_msg.quaternion_wxyz_e4[0] = veh_att.q1 * 1e4;
    copter_att_status_msg.quaternion_wxyz_e4[1] = veh_att.q2 * 1e4;
    copter_att_status_msg.quaternion_wxyz_e4[2] = veh_att.q3 * 1e4;
    copter_att_status_msg.quaternion_wxyz_e4[3] = veh_att.q4 * 1e4;
    copter_att_status_msg.reserved.len = 2;
    copter_att_status_msg.reserved.data[0] = 0;
    copter_att_status_msg.reserved.data[1] = 0;
    _detected_modules[_instance].ap_dronecan->xacti_copter_att_status.broadcast(copter_att_status_msg);
    return true;
}

// update zoom rate controller
// returns true if sent so that we avoid immediately trying to also send other messages
bool AP_Mount_Xacti::update_zoom_rate_control()
{
    // return immediately if zoom rate control is not enabled
    if (!_zoom_rate_control.enabled) {
        return false;
    }

    // update only every 0.5 sec
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _zoom_rate_control.last_update_ms < XACTI_ZOOM_RATE_UPDATE_INTERVAL_MS) {
        return false;
    }
    _zoom_rate_control.last_update_ms = now_ms;

    // increment zoom
    const uint16_t zoom_value = _last_zoom_param_value + _zoom_rate_control.increment;

    // if reached limit then disable zoom
    if ((zoom_value < 100) || (zoom_value > 1000)) {
        _zoom_rate_control.enabled = false;
        return false;
    }

    // send desired zoom to camera
    return set_param_int32(XACTI_PARAM_DIGITALZOOM, zoom_value);
}

// request firmware version
// returns true if sent so that we avoid immediately trying to also send other messages
bool AP_Mount_Xacti::request_firmware_version()
{
    // return immediately if already have version or no dronecan
    if (_firmware_version.received) {
        return false;
    }

    // send request once per second until received
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _firmware_version.last_request_ms < 1000) {
        return false;
    }
    _firmware_version.last_request_ms = now_ms;
    return get_param_string(XACTI_PARAM_FIRMWAREVERSION);
}

// request status
// returns true if sent so that we avoid immediately trying to also send other messages
bool AP_Mount_Xacti::request_status()
{
    // return immediately if 3 seconds has not passed
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _status_report.last_request_ms < XACTI_STATUS_REQ_INTERVAL_MS) {
        return false;
    }

    _status_report.last_request_ms = now_ms;
    return get_param_string(XACTI_PARAM_STATUS);
}

// check if safe to send message (if messages sent too often camera will not respond)
bool AP_Mount_Xacti::is_safe_to_send() const
{
    const uint32_t now_ms = AP_HAL::millis();

    // check time since last attitude sent
    if (now_ms - last_send_copter_att_status_ms < XACTI_MSG_SEND_MIN_MS) {
        return false;
    }

    // check time since last angle target sent
    if (now_ms - last_send_gimbal_control_ms < XACTI_MSG_SEND_MIN_MS) {
        return false;
    }

    // check time since last set param message sent
    if (now_ms - last_send_getset_param_ms < XACTI_MSG_SEND_MIN_MS) {
        return false;
    }

    return true;
}

#endif // HAL_MOUNT_XACTI_ENABLED
