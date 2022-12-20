add_executable(mycam
        MyArducam.cpp 
        XBee.cpp
        XBeePico.cpp
        )

# Pull in our pico_stdlib which pulls in commonly used features
include_directories(../ArduCAM)
target_link_libraries(mycam
	pico_stdlib
	hardware_dma
	hardware_i2c
	hardware_pwm
	ArduCAM
)
pico_enable_stdio_usb(mycam 1)
pico_enable_stdio_uart(mycam 0)
# create map/bin/hex file etc.
pico_add_extra_outputs(mycam)

