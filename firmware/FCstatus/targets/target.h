//Define all the targets that can be used

#if BOARD==ESP32DOIT_DEVKIT_V1
  #include "config_esp32doit.h"
#elif BOARD==ESP32_DEVKITC_V4
  #include "config_devkitcv4.h"
#else
  #error "Unknown board!"
#endif
