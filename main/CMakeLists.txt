idf_component_register(SRCS "led_strip_encoder.c" ## Task for LED
							# "my_espnow.c"
							"ledc_fade.c"		## Task for motor PWM control
							"pimotorcontrol.c" ## Main file
                       INCLUDE_DIRS ".")
