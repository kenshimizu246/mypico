add_executable(mycam
        MyArducam.cpp 
        XBee.cpp
        XBeePico.cpp
#tensorflow/lite/micro/tools/make/downloads/person_model_int8/person_image_data.cpp
#tensorflow/lite/micro/tools/make/downloads/person_model_int8/no_person_image_data.cpp 
#tensorflow/lite/micro/tools/make/downloads/person_model_int8/person_detect_model_data.cpp 
)

target_include_directories(mycam
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/.
  )

set_target_properties(
  mycam
  PROPERTIES
  COMPILE_FLAGS -fno-rtti
  COMPILE_FLAGS -fno-exceptions
  COMPILE_FLAGS -fno-threadsafe-statics
  COMPILE_FLAGS -nostdlib
)


target_sources(mycam
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/model_settings.cpp
  ${CMAKE_CURRENT_LIST_DIR}/tensorflow/lite/micro/tools/make/downloads/person_model_int8/person_detect_model_data.cpp
  ${CMAKE_CURRENT_LIST_DIR}/model_settings.h
  ${CMAKE_CURRENT_LIST_DIR}/person_detect_model_data.h
  )


# Pull in our pico_stdlib which pulls in commonly used features
include_directories(../ArduCAM)
target_link_libraries(mycam
        rp2040_arducam
	pico_stdlib
        pico_sync
	hardware_dma
	hardware_i2c
	hardware_pwm
	ArduCAM
)


pico_enable_stdio_usb(mycam 1)
pico_enable_stdio_uart(mycam 0)
# create map/bin/hex file etc.
pico_add_extra_outputs(mycam)


