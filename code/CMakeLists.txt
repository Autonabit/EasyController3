cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(myproject_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(easycontroller
	easycontroller.c
	)

target_link_libraries(easycontroller
	pico_stdlib
	pico_i2c_slave
	hardware_i2c
	hardware_pwm
	hardware_adc)

pico_enable_stdio_usb(easycontroller 1)	# Enable USB printf output
pico_enable_stdio_uart(easycontroller 0)	# Disable serial printf output

pico_add_extra_outputs(easycontroller)

