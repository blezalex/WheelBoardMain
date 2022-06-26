
#include "settings.hpp"
#include "drv/comms/config.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include "stm32f10x_flash.h"

#define CONFIG_SIZE_MAX 512
#define CONFIG_FLASH_PAGE_ADDR (FLASH_BASE + 128*1024 - CONFIG_SIZE_MAX)


int32_t saveProtoToBuffer(uint8_t* buffer, int16_t max_size, const pb_msgdesc_t* fields, const void *src_struct,  Usart* log ) {
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, max_size);
	if (!pb_encode(&stream, fields, src_struct)) {
		if (log != nullptr)
		{
			log->Send(PB_GET_ERROR(&stream), strlen(PB_GET_ERROR(&stream)));
		}
		return -1;
	}

	return stream.bytes_written;
}

uint8_t scratch[CONFIG_SIZE_MAX];

bool saveSettingsToFlash(const Config& config) {
	int32_t size = saveProtoToBuffer(scratch, sizeof(scratch), Config_fields, &config);
	if (size < 0)
		return false;

	FLASH_Unlock();
	if (FLASH_ErasePage(CONFIG_FLASH_PAGE_ADDR) != FLASH_Status::FLASH_COMPLETE)
		return false;

	if (FLASH_ProgramWord((uint32_t)CONFIG_FLASH_PAGE_ADDR, size) != FLASH_Status::FLASH_COMPLETE)
		return false;

	int size_round_up = (size + 3) / 4 * 4;
	for (int i = 0; i < size_round_up/ 4; i++) {
		// +4 for size block
		if (FLASH_ProgramWord((uint32_t)CONFIG_FLASH_PAGE_ADDR + i*4 + 4, ((uint32_t*)scratch)[i]) != FLASH_Status::FLASH_COMPLETE)
			return false;
	}

	FLASH_Lock();
	return true;
}

bool readSettingsFromBuffer(Config* config, const uint8_t* data, uint32_t data_len) {
	pb_istream_t stream = pb_istream_from_buffer(data, data_len);

	return pb_decode(&stream, Config_fields, config);
}

bool readSettingsFromFlash(Config* config) {
	uint32_t size = *(uint32_t*)CONFIG_FLASH_PAGE_ADDR;
	return readSettingsFromBuffer(config, (const uint8_t*)(CONFIG_FLASH_PAGE_ADDR + 4), size);
}


