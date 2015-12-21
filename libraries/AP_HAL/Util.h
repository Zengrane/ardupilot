
#ifndef __AP_HAL_UTIL_H__
#define __AP_HAL_UTIL_H__

#include <stdarg.h>
#include "AP_HAL_Namespace.h"

class AP_HAL::Util {
public:
    int snprintf(char* str, size_t size,
                 const char *format, ...);

    int vsnprintf(char* str, size_t size,
                  const char *format, va_list ap);

    enum soft_arm_state_t {
        SOFT_ARM_STATE_DISARMED=0,
        SOFT_ARM_STATE_ARMING,
        SOFT_ARM_STATE_ARMED
    };

    void set_soft_arm_state(const soft_arm_state_t state) { _soft_arm_state = state; }
    enum soft_arm_state_t get_soft_arm_state() const { return _soft_arm_state; }

    void set_capabilities(uint64_t cap) { capabilities |= cap; }
    void clear_capabilities(uint64_t cap) { capabilities &= ~(cap); }
    uint64_t get_capabilities() const { return capabilities; }

    virtual const char* get_custom_log_directory() { return NULL; } 
    virtual const char* get_custom_terrain_directory() const { return NULL;  }

    // run a debug shall on the given stream if possible. This is used
    // to support dropping into a debug shell to run firmware upgrade
    // commands
    virtual bool run_debug_shell(AP_HAL::BetterStream *stream) = 0;

    enum safety_state {
        SAFETY_NONE, SAFETY_DISARMED, SAFETY_ARMED
    };

    /*
      return state of safety switch, if applicable
     */
    virtual enum safety_state safety_switch_state(void) { return SAFETY_NONE; }

    /*
      set system clock in UTC microseconds
     */
    virtual void set_system_clock(uint64_t time_utc_usec) {}

    /*
      get system identifier (eg. serial number)
      return false if a system identifier is not available

      Buf should be filled with a printable string and must be null
      terminated
     */
    virtual bool get_system_id(char buf[40]) { return false; }

    /**
       how much free memory do we have in bytes. If unknown return 4096
     */
    virtual uint32_t available_memory(void) { return 4096; }

    /**
       return commandline arguments, if available
     */
    virtual void commandline_arguments(uint8_t &argc, char * const *&argv) { argc = 0; }
    
    /*
        ToneAlarm Driver
    */
    virtual bool toneAlarm_init() { return false;}
    virtual void toneAlarm_set_tune(uint8_t tune) {}
    virtual void _toneAlarm_timer_tick() {}

    /*
      return a stream for access to a system shell, if available
     */
    virtual AP_HAL::Stream *get_shell_stream() { return NULL; }

    /* Support for an imu heating system */
    virtual void set_imu_temp(float current) {}

    /*
      performance counter calls - wrapper around original PX4 interface
     */
    enum perf_counter_type {
	PC_COUNT,		/**< count the number of times an event occurs */
	PC_ELAPSED,		/**< measure the time elapsed performing an event */
	PC_INTERVAL		/**< measure the interval between instances of an event */
    };
    typedef void *perf_counter_t;
    virtual perf_counter_t perf_alloc(perf_counter_type t, const char *name) { return NULL; }
    virtual void perf_begin(perf_counter_t h) {}
    virtual void perf_end(perf_counter_t h) {}
    virtual void perf_count(perf_counter_t h) {}

    // create a new semaphore
    virtual Semaphore *new_semaphore(void) { return nullptr; }
    
protected:
    // we start soft_armed false, so that actuators don't send any
    // values until the vehicle code has fully started
    enum soft_arm_state_t _soft_arm_state = SOFT_ARM_STATE_DISARMED;
    uint64_t capabilities = 0;
};

#endif // __AP_HAL_UTIL_H__

