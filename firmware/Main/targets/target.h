//Define all the targets that can be used

#if BOARD==ESP32DOIT_DEVKIT_V1
  #include "config_esp32doit.h"
#else
  #error "Unknown board!"
#endif
