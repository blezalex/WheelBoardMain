
#include "settings.hpp"
#include "drv/comms/protocol.pb.h"
#include "../nanopb-0.3.9.2-windows-x86/pb_encode.h"
#include "../nanopb-0.3.9.2-windows-x86/pb_decode.h"
#include "stm32f10x_flash.h"

#define CONFIG_SIZE_MAX 512
#define CONFIG_FLASH_PAGE_ADDR (FLASH_BASE + 128*1024 - CONFIG_SIZE_MAX)

bool saveToBufferFn(pb_ostream_t *stream, const uint8_t *buf, size_t count) {
	uint8_t** out = (uint8_t**)stream->state;

	for (size_t i = 0; i < count; i++) {
		*((*out)++) = *(buf++);
	}
	return true;
}

int32_t saveProtoToBuffer(uint8_t* buffer, int16_t max_size, const pb_field_t fields[], const void *src_struct,  Usart* log ) {
	pb_ostream_t sizestream = { 0 };
	pb_encode(&sizestream, fields, src_struct);

	if (sizestream.bytes_written > max_size) {
		return -sizestream.bytes_written;
	}

	pb_ostream_t save_stream = { &saveToBufferFn, &buffer, max_size, 0 };
	if (!pb_encode(&save_stream, fields, src_struct)) {
		if (log != nullptr)
		{
			log->Send(save_stream.errmsg, strlen(save_stream.errmsg));

		}
		return -1;
	}

	return sizestream.bytes_written;
}

bool saveSettingsToFlash(const Config& config) {
	uint8_t buffer[255];
	int32_t size = saveProtoToBuffer(buffer, sizeof(buffer), Config_fields, &config);
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
		if (FLASH_ProgramWord((uint32_t)CONFIG_FLASH_PAGE_ADDR + i*4 + 4, ((uint32_t*)buffer)[i]) != FLASH_Status::FLASH_COMPLETE)
			return false;
	}

	FLASH_Lock();
	return true;
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


