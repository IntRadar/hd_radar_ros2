#ifndef CRC16_CCITT_HPP
#define CRC16_CCITT_HPP

#include <cstdint>

static inline uint16_t crc16_ccitt_init(void) 
{
	return 0xFFFF;
}

static inline uint16_t crc16_ccitt_update(uint8_t byte, uint16_t crc) 
{
	int i;
	int xor_flag;

	/* For each bit in the data byte, starting from the leftmost bit */
	for (i = 7; i >= 0; i--) {
		/* If leftmost bit of the CRC is 1, we will XOR with
		 * the polynomial later */
		xor_flag = crc & 0x8000;

		/* Shift the CRC, and append the next bit of the
		 * message to the rightmost side of the CRC */
		crc <<= 1;
		crc |= (byte & (1 << i)) ? 1 : 0;

		/* Perform the XOR with the polynomial */
		if (xor_flag)
			crc ^= 0x1021;
	}

	return crc;
}

static inline uint16_t crc16_ccitt_finalize(uint16_t crc) 
{
	int i;

	/* Augment 16 zero-bits */
	for (i = 0; i < 2; i++) {
		crc = crc16_ccitt_update(0, crc);
	}

	return crc;
}

static inline uint16_t CRC16Get(void *data, std::size_t len) 
{
	uint8_t *b = (uint8_t *)data;
	std::size_t i;
	uint16_t crc;

	crc = crc16_ccitt_init();

	/* Update the CRC using the data */
	for (i = 0; i < len; i++)
		crc = crc16_ccitt_update(b[i], crc);

	crc = crc16_ccitt_finalize(crc);

	return crc;
}

#endif