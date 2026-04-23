idf_component_register(SRCS "led_strip_encoder.c" ## Task for LED
							"my_espnow.c" ## Joystick comms 
							"control_loop.c" 
							"ledc_fade.c"		## Task for motor PWM control
							"claude_i2c_est.c"
							"pimotorcontrol.c" ## Main file
                       INCLUDE_DIRS ".")
