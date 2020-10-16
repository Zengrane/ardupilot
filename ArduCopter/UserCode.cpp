#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up


	//Servo Voltage Watcher
	AP_Notify::flags.low_servo_voltage = false;


	//Killswitch Variables
	killswitch_counter = 0;


	//Herelink Gimbal Control Variables
	ap.gimbal_control_active = false;
	speed_setting = 1;
	function_counter = 0;
	cam_button_debounce_timer = 0;

	ch9_button_pressed = false;
	ch10_button_pressed = false;
	ch11_button_pressed = false;
	ch12_button_pressed = false;

	long_press_flag_ch9 = false;
	long_press_flag_ch10 = false;
	long_press_flag_ch11 = false;
	long_press_flag_ch12 = false;

	short_press_flag_ch9 = false;
	short_press_flag_ch10 = false;
	short_press_flag_ch11 = false;
	short_press_flag_ch12 = false;

	ch9_button_hold = false;
	ch10_button_hold = false;
	ch11_button_hold = false;
	ch12_button_hold = false;

	SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1500);
	SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1500);
	SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_zoom, 1500);
	SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_tilt, 1500);
	SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_pan, 1500);


	// startup spirit state
	spirit_state = disarm;


	//GPIOs for on/off to payload and critical systems

	hal.gpio->pinMode(52, 1);  // Payload, FLASE to turn on
	hal.gpio->pinMode(53, 1);  // ESCs/Servos, TRUE to turn on

	hal.gpio->write(52, false);	// Payload Power off at startup
	hal.gpio->write(53, true);  //Turn ESC/Servos on at startup

	//Spool up support
	timer_trigger = false;

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here


}


#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{

	//detect button input in Herelink at 50Hz
	if(g.herelink_enable){
		Detect_Buttons();
	}

	//Call topple sense

	topple_sense();


	// State Machine

	if(!motors->armed()){
		spirit_state = disarm;

	// the rest considers the vehicle to be armed
		//always move through spoolup
	}else if(spirit_state == disarm){

		hal.gpio->write(52, false);
		hal.gpio->write(53, true);

		spirit_state = spoolup;
		spoolup_watcher = AP_HAL::millis16();
	//	gcs().send_text(MAV_SEVERITY_INFO,"SpoolUp");

	}else if(spirit_state == land and (copter.flightmode->is_taking_off() or !ap.land_complete)){

		spirit_state = takeoff;
	//	gcs().send_text(MAV_SEVERITY_INFO,"takeoff");

	}else if(spirit_state == takeoff and (!copter.flightmode->is_taking_off() or copter.flightmode->has_manual_throttle()) ){

		spirit_state = hover;
	//	gcs().send_text(MAV_SEVERITY_INFO,"hover");


	}else if((spirit_state == hover or spirit_state == landing) and ap.land_complete){

		spirit_state = land;
	//	gcs().send_text(MAV_SEVERITY_INFO,"land");

	}




	switch(spirit_state){


	case disarm:

		//zero out dynamic trim for now
		motors->set_dynamic_trim(0.0, 0.0);

		if(RC_Channels::rc_channel(CH_3)->get_radio_in() == 0){
			AP_Notify::flags.no_RC_in = true;
		}else{
			AP_Notify::flags.no_RC_in = false;
		}

		break;

	case spoolup:

		//Looks for issues with startup
		servo_voltage_watcher();

		//Spoolup Watcher.  Disarm if 4 seconds passes without advancing
		if(AP_HAL::millis16() - spoolup_watcher > (uint16_t)4000){
			 copter.arming.disarm();
			 gcs().send_text(MAV_SEVERITY_CRITICAL,"A Rotor Failed to Start: Disarm");
		}

		_fwd_rpm = rpm_sensor.get_rpm(0);
		_aft_rpm = rpm_sensor.get_rpm(1);

		if(_fwd_rpm > 1000.0f){

			if(!timer_trigger){
				spoolup_timer =  AP_HAL::millis16();
				spoolup_watcher = AP_HAL::millis16();
				timer_trigger = true;
			}

			if(_fwd_rpm >= 1000.0f and _aft_rpm >= 1000.0f){  //fully spun up
				motors->spoolup_complete(true);
				spirit_state = land;
				//gcs().send_text(MAV_SEVERITY_INFO,"landed");
				timer_trigger = false;

			}else if((AP_HAL::millis16() - spoolup_timer) > (uint16_t)(g.spool_delta*1000)){
				motors->enable_aft_rotor(true);
			}else{
				spoolup_watcher = AP_HAL::millis16();
			}
		}

        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_alt_hold_controllers(0.0f);

		break;

	case takeoff:

		break;

	case land:

		break;

	case hover:

		//////// Determine if we are landing //////
		if(copter.flightmode->get_alt_above_ground_cm() <= (int32_t)g2.land_alt_low){   // need to be close to the ground

			if(copter.flightmode->is_landing()){
				spirit_state = landing;
				attitude_control->enable_angle_boost(false);
				//gcs().send_text(MAV_SEVERITY_INFO,"L:auto");
				break;
			// for manual also need to be decending
			}else if(!copter.flightmode->is_autopilot() and copter.flightmode->get_pilot_desired_climb_rate((float)channel_throttle->get_control_in()) < 0.0f){
				spirit_state = landing;
				attitude_control->enable_angle_boost(false);
				//gcs().send_text(MAV_SEVERITY_INFO,"L:dct");
				break;
			}

		}

		break;

	case landing:

		//////cancel landing if no longer auto landing or if pilot is not commanding a decent
			if(!copter.flightmode->is_autopilot() and copter.flightmode->get_pilot_desired_climb_rate(channel_throttle->get_control_in()) >= 0.0f){
				spirit_state = hover;
				attitude_control->enable_angle_boost(true);
				//gcs().send_text(MAV_SEVERITY_INFO,"H:cr");
				break;
			}

			if(copter.flightmode->is_autopilot() and !copter.flightmode->is_landing()){
				spirit_state = hover;
				attitude_control->enable_angle_boost(true);
				//gcs().send_text(MAV_SEVERITY_INFO,"H:!land");
				break;
			}

		break;

		}

}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here

	Killswitch();


	if(!motors->armed()){
		return;
	}


}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here


	if(g.herelink_enable){
		Decode_Buttons();
	}else{
		copter.ap.gimbal_control_active = false;
	}


}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here

}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif




void Copter::topple_sense(){

		if(spirit_state == spoolup){
			if(labs(ahrs.pitch_sensor) >1500 or labs(ahrs.roll_sensor) > 1500){
				 copter.arming.disarm();
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple spoolup");
			}

		}else if(spirit_state == takeoff){
			if(fabsf(attitude_control->get_att_error_angle_deg()) > 30.0f and copter.flightmode->get_alt_above_ground_cm() < 150){
				 copter.arming.disarm();
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple takeoff");
			}

		}else if(spirit_state == landing){

			if(fabsf(attitude_control->get_att_error_angle_deg()) > 45.0f){
				copter.arming.disarm();
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple landing");
				 return;
			}

			if(fabsf(attitude_control->get_att_error_angle_deg()) > 30.0f and channel_throttle->get_control_in() == 0){
				 copter.arming.disarm();
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple landing");
			}

			if((RangeFinder::RangeFinder_OutOfRangeLow or rangefinder_state.alt_cm_filt.get() <= 75.0f) and fabsf(attitude_control->get_att_error_angle_deg()) > 20.0f and copter.flightmode->get_alt_above_ground_cm() < 400){
				 copter.arming.disarm();
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple landing");
			}

		}else{

			/////Always the case
			if(fabsf(attitude_control->get_att_error_angle_deg()) > 100.0f and copter.flightmode->get_alt_above_ground_cm() < 400){
				 copter.arming.disarm();
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple crash");
			}

		}
}



void Copter::servo_voltage_watcher(){

		 const float servo_voltage = hal.analogin->servorail_voltage();

		 if(servo_voltage < 4.0 ){
			 copter.arming.disarm();
			 AP_Notify::flags.low_servo_voltage = true;
			 gcs().send_text(MAV_SEVERITY_CRITICAL,"LOW SERVO VOLTAGE");
			}
}



void Copter::Spirit_Gimbal_Control_Auto(){

	if(!g.herelink_enable){
		return;
	}

int16_t  gimbal_pan, gimbal_tilt, gimbal_zoom, gimbal_focus;

		gimbal_tilt = RC_Channels::rc_channel(CH_2)->get_radio_in();
		gimbal_pan = RC_Channels::rc_channel(CH_1)->get_radio_in();
		gimbal_zoom = RC_Channels::rc_channel(CH_3)->get_radio_in();
		gimbal_focus = RC_Channels::rc_channel(CH_4)->get_radio_in();

		SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_tilt, gimbal_tilt);
		SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_pan, gimbal_pan);
		SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_zoom, gimbal_zoom);
		SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_focus, gimbal_focus);
}



void Copter::Killswitch(){

	if(!motors->armed()){
		killswitch_counter = 0;
		return;
	}

	if(RC_Channels::rc_channel(CH_8)->get_radio_in() < 1700){
			killswitch_counter = 0;
			return;
	}


	//todo: killswitch activation time dependent on particular condition (ie flying versus toppling)

	bool killswitch_activate = {
			ap.throttle_zero or
			ap.land_complete or
			channel_throttle->get_control_in() == 0 or
			fabsf(attitude_control->get_att_error_angle_deg()) > 45.0f};

	 if(killswitch_activate) {
			 if(killswitch_counter >= 2){
				 copter.arming.disarm();
				 killswitch_counter = 0;
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"User: Disarm");

			 }else{ killswitch_counter++;  }

	 }else{   killswitch_counter = 0;  }

}



void Copter::Detect_Buttons(){

	if(RC_Channels::rc_channel(CH_9)->get_radio_in() > 1800){
				if(!ch9_button_hold){
					if(!ch9_button_pressed){
						ch9_button_pressed = true;
						ch9_timer = millis();
					}else{
						if( (millis() - ch9_timer) > 750 ){
							long_press_flag_ch9 = true;  //these are reset in the 10Hz loop
							function_counter = 0;
							ch9_button_hold = true;
						}
					}
				}
			}else{
				if(ch9_button_pressed){
					if(!ch9_button_hold){  //if hold was active don't do a short_press
						short_press_flag_ch9 = true;//these are reset in the 10Hz loop
						function_counter = 0;
					}
					ch9_button_hold = false;
					ch9_button_pressed = false; //reset button press flag
				}
			}


		if(RC_Channels::rc_channel(CH_10)->get_radio_in() > 1800){
					if(!ch10_button_hold){
						if(!ch10_button_pressed){
							ch10_button_pressed = true;
							ch10_timer = millis();
						}else{
							if( (millis() - ch10_timer) > 750 ){
								long_press_flag_ch10 = true;  //these are reset in the 10Hz loop
								function_counter = 0;
								ch10_button_hold = true;
							}
						}
					}
				}else{
					if(ch10_button_pressed){
						if(!ch10_button_hold){  //if hold was active don't do a short_press
							short_press_flag_ch10 = true;//these are reset in the 10Hz loop
							function_counter = 0;
						}
						ch10_button_hold = false;
						ch10_button_pressed = false; //reset button press flag
					}
				}


	if(RC_Channels::rc_channel(CH_11)->get_radio_in() > 1800){
				if(!ch11_button_hold){
					if(!ch11_button_pressed){
						ch11_button_pressed = true;
						ch11_timer = millis();
					}else{
						if( (millis() - ch11_timer) > 750 ){
							long_press_flag_ch11 = true;  //these are reset in the 10Hz loop
							function_counter = 0;
							ch11_button_hold = true;
						}
					}
				}
			}else{
				if(ch11_button_pressed){
					if(!ch11_button_hold){  //if hold was active don't do a short_press
						short_press_flag_ch11 = true;//these are reset in the 10Hz loop
						function_counter = 0;
					}
					ch11_button_hold = false;
					ch11_button_pressed = false; //reset button press flag
				}
			}
		}



void Copter::Decode_Buttons(){

	if(RC_Channels::rc_channel(CH_7)->get_radio_in() >= 1700){
		copter.ap.gimbal_control_active = true;
		cam_button_pressed = true;
	}else if(cam_button_pressed){
		if(cam_button_debounce_timer >= 1){
			cam_button_pressed = false;
			cam_button_debounce_timer = 0;
		}else{
			cam_button_debounce_timer++;
		}
	}else{
		copter.ap.gimbal_control_active = false;
	}

	if(short_press_flag_ch9){
			if(function_counter < 2){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1500);
				function_counter++;
			}else if(function_counter < 4){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1100);
				function_counter++;
			}else{
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1500);
				short_press_flag_ch9 = false;
				function_counter = 0;
			}
		}



		if(long_press_flag_ch9){
			if(function_counter < 2){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1500);
				function_counter++;
			}else if(function_counter < 4){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1900);
				function_counter++;
			}else{
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1500);
				long_press_flag_ch9 = false;
				function_counter = 0;
			}

		}

		if(short_press_flag_ch10){

			short_press_flag_ch10 = false;

			if(speed_setting == 3){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1900);
			}else if(speed_setting == 1){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1100);
			}else{
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1500);
			}
			speed_setting++;
			if(speed_setting > 3){ speed_setting = 1; }
		}

		if(long_press_flag_ch10){
			if(function_counter < 4){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1500);
				function_counter++;
			}else if(function_counter < 5){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1900);
				function_counter++;
			}else{
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1500);
				function_counter++;
				long_press_flag_ch10 = false;
				function_counter = 0;
			}
		}

		if(short_press_flag_ch11){
			short_press_flag_ch11 = false;
			if(!flight_mode_switch){
				copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND);
				 AP_Notify::events.user_mode_change = 1;
				flight_mode_switch = true;
			}else{
				if(!copter.set_mode((Mode::Number)copter.flight_modes[2].get(), ModeReason::GCS_COMMAND)){
					 AP_Notify::events.user_mode_change_failed = 1;
				}else{
					AP_Notify::events.user_mode_change = 1;
				}

				flight_mode_switch = false;
			}
		}

		if(long_press_flag_ch11){
			long_press_flag_ch11 = false;
			copter.set_mode(Mode::Number::BRAKE, ModeReason::GCS_COMMAND);
		}

}

