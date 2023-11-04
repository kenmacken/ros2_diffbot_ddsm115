#ifndef __CRC_H__
#define __CRC_H__

#include <iostream>

uint8_t reverseBits(uint8_t value) {
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        result = (result << 1) | ((value >> i) & 1);
    }
    return result;
}

uint16_t reverseBits(uint16_t value) {
    uint16_t result = 0;
    for (int i = 0; i < 16; i++) {
        result = (result << 1) | ((value >> i) & 1);
    }
    return result;
}

uint32_t reverseBits(uint32_t value) {
    uint32_t result = 0;
    for (int i = 0; i < 32; i++) {
        result = (result << 1) | ((value >> i) & 1);
    }
    return result;
}

uint8_t crc8(uint8_t* data, size_t length, uint8_t poly, uint8_t init, bool refIn, bool refOut, uint8_t xorOut) {
	
    uint8_t crc = init;
    
	if (refIn && refOut){
		crc = reverseBits(crc);
		poly = reverseBits(poly);
		for (size_t i = 0; i < length; i++) {
			crc ^= data[i];
			for (int j = 0; j < 8; j++) crc = (crc & 0x01) ? (crc >> 1) ^ poly : crc >> 1;
		}
		crc = crc ^ xorOut;
	} else {
		for (size_t i = 0; i < length; i++) {
			crc ^= refIn ? reverseBits(data[i]) : data[i];
			for (int j = 0; j < 8; j++) crc = (crc & 0x80) ? (crc << 1) ^ poly : crc << 1;
		}
		crc = refOut ? reverseBits(crc) ^ xorOut : crc ^ xorOut;
	}
    return crc;
}


uint16_t crc16(uint8_t* data, size_t length, uint16_t poly, uint16_t init, bool refIn, bool refOut, uint16_t xorOut) {
	
    uint16_t crc = init;
    
	if (refIn && refOut){
		crc = reverseBits(crc);
		poly = reverseBits(poly);
		for (size_t i = 0; i < length; i++) {
			crc ^= data[i];
			for (int j = 0; j < 8; j++) crc = (crc & 0x01) ? (crc >> 1) ^ poly : crc >> 1;
		}
		crc = crc ^ xorOut;
	} else {
		for (size_t i = 0; i < length; i++) {
			crc ^= refIn ? ((uint16_t)reverseBits(data[i]) << 8) : ((uint16_t)data[i] << 8);
			for (int j = 0; j < 8; j++) crc = (crc & 0x8000) ? (crc << 1) ^ poly : crc << 1;
		}
		crc = refOut ? reverseBits(crc) ^ xorOut : crc ^ xorOut;
	}
    return crc;
}

uint32_t crc32(uint8_t* data, size_t length, uint32_t poly, uint32_t init, bool refIn, bool refOut, uint32_t xorOut) {
	
    uint32_t crc = init;
    
	if (refIn && refOut){
		crc = reverseBits(crc);
		poly = reverseBits(poly);
		for (size_t i = 0; i < length; i++) {
			crc ^= data[i];
			for (int j = 0; j < 8; j++) crc = (crc & 0x01) ? (crc >> 1) ^ poly : crc >> 1;
		}
		crc = crc ^ xorOut;
	} else {
		for (size_t i = 0; i < length; i++) {
			crc ^= refIn ? ((uint32_t)reverseBits(data[i]) << 24) : ((uint32_t)data[i] << 24);
			for (int j = 0; j < 8; j++) crc = (crc & 0x80000000) ? (crc << 1) ^ poly : crc << 1;
		}
		crc = refOut ? reverseBits(crc) ^ xorOut : crc ^ xorOut;
	}
    return crc;
}

#endif // __CRC_H__