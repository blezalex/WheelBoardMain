
#include "settings.hpp"
#include "drv/comms/protocol.pb.h"
#include "../nanopb-0.3.9.2-windows-x86/pb_encode.h"
#include "../nanopb-0.3.9.2-windows-x86/pb_decode.h"
#include "stm32f10x_flash.h"

#define CONFIG_SIZE_MAX 1024
#define CONFIG_FLASH_PAGE_ADDR (FLASH_BASE + 128*1024 - CONFIG_SIZE_MAX)


struct save_state_t {
	uint32_t data;
	uint8_t data_size;
	uint16_t flash_offset;
};

bool saveFn(pb_ostream_t *stream, const uint8_t *buf, size_t count) {
	save_state_t* state = (save_state_t*)stream->state;

	uint8_t* data_ptr = (uint8_t*)&state->data;
	for (size_t i = 0; i < count; i++) {
		*(data_ptr + state->data_size++) = *(buf++);

		if (state->data_size == 4) {
			FLASH_ProgramWord((uint32_t)CONFIG_FLASH_PAGE_ADDR + state->flash_offset, state->data);
			state->data_size = 0;
			state->data = 0;
			state->flash_offset += 4;
		}
	}
	return true;
}

bool saveSettingsToFlash(const Config& config) {
	pb_ostream_t sizestream = { 0 };
	pb_encode(&sizestream, Config_fields, &config);

	FLASH_Unlock();
	FLASH_ErasePage(CONFIG_FLASH_PAGE_ADDR);

	save_state_t save_state = { 0, 0, 4 };
	pb_ostream_t save_stream = { &saveFn, &save_state, CONFIG_SIZE_MAX, 0 };

	uint32_t size = sizestream.bytes_written;
	FLASH_ProgramWord((uint32_t)CONFIG_FLASH_PAGE_ADDR, size);
	if (pb_encode(&save_stream, Config_fields, &config)) {
		if (save_state.data_size > 0) {
			// save last block (incomplete block)
			FLASH_ProgramWord((uint32_t)CONFIG_FLASH_PAGE_ADDR + save_state.flash_offset, save_state.data);
		}
	}

	FLASH_Lock();
	return true;
}

bool saveToBufferFn(pb_ostream_t *stream, const uint8_t *buf, size_t count) {
	uint8_t** out = (uint8_t**)stream->state;

	for (size_t i = 0; i < count; i++) {
		*((*out)++) = *(buf++);
	}
	return true;
}

int16_t saveSettingsToBuffer(uint8_t* buffer, int16_t max_size, const Config& config) {
	pb_ostream_t sizestream = { 0 };
	pb_encode(&sizestream, Config_fields, &config);

	if (sizestream.bytes_written > max_size)
		return -1;

	pb_ostream_t save_stream = { &saveToBufferFn, &buffer, CONFIG_SIZE_MAX, 0 };
	if (!pb_encode(&save_stream, Config_fields, &config))
		return -1;

	return sizestream.bytes_written;
}

bool read_fn(pb_istream_t *stream, uint8_t *buf, size_t count) {
	uint32_t* read_offset = (uint32_t*)stream->state;
//	if (*read_offset + count > (CONFIG_FLASH_PAGE_ADDR + CONFIG_SIZE_MAX))
//		return false;

	memcpy(buf, (uint8_t*)*read_offset, count);
	*read_offset += count;
	return true;
}

bool readSettingsFromBuffer(Config* config, const uint8_t* data, uint32_t data_len) {
	uint32_t read_offset = (uint32_t)data;
	pb_istream_t stdinstream = { &read_fn, &read_offset, data_len };

	return pb_decode(&stdinstream, Config_fields, config);
}

bool readSettingsFromFlash(Config* config) {
	uint32_t size = *(uint32_t*)CONFIG_FLASH_PAGE_ADDR;
	return readSettingsFromBuffer(config, (const uint8_t*)(CONFIG_FLASH_PAGE_ADDR + 4), size);
}


