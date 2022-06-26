#pragma once

#include "drv/comms/config.pb.h"
#include "io/usart.hpp"

bool saveSettingsToFlash(const Config& config);

bool readSettingsFromFlash(Config* config);

bool readSettingsFromBuffer(Config* config, const uint8_t* data, uint32_t data_len);

int32_t saveProtoToBuffer(uint8_t* buffer, int16_t max_size, const pb_msgdesc_t* fields, const void *src_struct, Usart* log = nullptr);
