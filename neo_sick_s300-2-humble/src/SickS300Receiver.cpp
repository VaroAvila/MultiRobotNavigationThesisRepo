/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "../include/SickS300Receiver.h"

#include <cstring>
#include <stdexcept>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <linux/serial.h>
#include <arpa/inet.h>


#define MAX_TELEGRAM_BYTES 2048

const unsigned short crc_table[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
	0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
	0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
	0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
	0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
	0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
	0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
	0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
	0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
	0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
	0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
	0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
	0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
	0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
	0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
	0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
	0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
	0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
	0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
	0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
	0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
	0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
	0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

static uint16_t compute_crc(const uint8_t* p_data, ssize_t num_bytes);


SickS300Receiver::SickS300Receiver()
{
	m_buffer.resize(MAX_TELEGRAM_BYTES * 2);
}

SickS300Receiver::~SickS300Receiver()
{
	if(m_fd >= 0) {
		::close(m_fd);
	}
}

void SickS300Receiver::open(const std::string& port, int baud_rate)
{
	if(m_fd >= 0) {
		::close(m_fd);
	}

	// open serial port
	m_fd = ::open(port.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);

	if(m_fd < 0) {
		throw std::runtime_error("open() failed with: " + std::string(std::strerror(errno)));
	}

	int baud_code = 0;
	switch(baud_rate)
	{
		case 115200: baud_code = B115200; break;
		case 230400: baud_code = B230400; break;
		case 500000: baud_code = B500000; break;
		case 1000000: baud_code = B1000000; break;
		default: throw std::logic_error("unsupported baudrate");
	}

	// configure serial port
	termios options = {};
	tcgetattr(m_fd, &options); // get current config

	cfmakeraw(&options); // configure for binary data

	options.c_cflag &= ~CSTOPB;		   // one stop bit
	options.c_cflag &= ~CRTSCTS;	   // no hardware handshake
	options.c_cflag |= CLOCAL | CREAD; // legacy magic

	options.c_cc[VMIN] = 1;  // read at least one byte per read() call
	options.c_cc[VTIME] = 0; // inter-byte timeout (not needed in this case)

	cfsetispeed(&options, baud_code); // input baudrate

	tcsetattr(m_fd, TCSANOW, &options); // apply the new config
}

/**
 * S300 header format in continuous mode:
 *
 *      | 00 00 00 00 |   4 byte reply header
 *
 *		Now starts the actual telegram
 *
 *      | 00 00 |         data block number (fixed)
 *      | xx xx |         size of data telegram (should be dec 1104)
 *      | FF xx |         last byte decides scanner id, 07 in most cases, but 08 for slave configured scanners
 *      | xx xx |         protocol version
 *      | 0x 00 |         status: 00 00 = normal, 01 00 = lockout
 *      | xx xx xx xx |   scan number
 *      | xx xx |         telegram number
 *      | BB BB |         fixed (ID for distance data)
 *      | 11 11 |         fixed (ID for first measurement area)
 *	       ...            data
 *      | xx xx |         CRC
 *
 *	   Telegram (as it is stored in the Buffer):	Position in the telegram
 *	   Header: 		bytes 4 to 23 = 20 bytes		bytes 0 to 19
 *	   Data:   		bytes 24 to 1105 = 1082 bytes	bytes 20 to 1101
 *	   CRC:    		bytes 1106, 1107 = 2 bytes		bytes 1102, 1103
 */

void SickS300Receiver::read(int timeout_ms)
{
	// try to read data
	{
		fd_set read_set;
		FD_ZERO(&read_set);
		FD_SET(m_fd, &read_set);

		// wait for input, with a timeout
		struct timeval timeout = {(timeout_ms / 1000), (timeout_ms % 1000) * 1000};
		const int res = select(m_fd + 1, &read_set, 0, 0, &timeout) == 1;

		if(res == 0) {
			handle_debug_msg("select() timeout");
			return;		// timeout
		}
		if(res < 0) {
			throw std::runtime_error("select() error: " + std::string(std::strerror(errno)));
		}

		// check for buffer overflow (should never happen)
		if( (size_t)m_offset >= m_buffer.size()) {
			handle_debug_msg("buffer overflow");
			m_offset = 0;	// discard all data
		}

		// read data
		const int num_bytes = ::read(m_fd, m_buffer.data() + m_offset, m_buffer.size() - m_offset);

		if(num_bytes < 0) {
			throw std::runtime_error("read() error: " + std::string(std::strerror(errno)));
		}
		m_offset += num_bytes;
	}

	// process data
	while(true)
	{
		// find header
		bool found_header = false;
		for(ssize_t i = 0; i < m_offset - 10; ++i)
		{
			if(m_buffer[i + 0] == 0 && m_buffer[i + 1] == 0 && m_buffer[i + 2] == 0 && m_buffer[i + 3] == 0
				&& m_buffer[i + 4] == 0 && m_buffer[i + 5] == 0 && m_buffer[i + 8] == 0xFF)
			{
				// remove preceeding data (since it's an incomplete message)
				discard_bytes(i);
				found_header = true;
				break;
			}
		}

		if(!found_header) {
			break;
		}

		// get pointer to beginning of telegram payload
		const uint8_t* const p_data = m_buffer.data() + 4;
		const ssize_t num_bytes_avail = m_offset - 4;

		// read telegram size (in units of words/two-bytes)
		uint16_t telegram_size = 0;
		::memcpy(&telegram_size, p_data + 2, 2);
		telegram_size = ntohs(telegram_size);

		// read version number
		uint16_t protocol_version = 0;
		::memcpy(&protocol_version, p_data + 6, 2);

		// check version
		if(protocol_version != 0x0102 && protocol_version != 0x0103)
		{
			discard_bytes(4);	// discard invalid bytes
			handle_debug_msg("protocol_version != 0x0102 | 0x0103 (actual = " + std::to_string(protocol_version) + ")");
			continue;
		}

		if(protocol_version == 0x0103)
		{
			telegram_size += 5;
		}

		const ssize_t telegram_bytes = telegram_size * 2;

		// check if valid telegram
		if(telegram_bytes > MAX_TELEGRAM_BYTES)
		{
			discard_bytes(4);	// discard invalid bytes
			handle_debug_msg("telegram_bytes > MAX_TELEGRAM_BYTES");
			continue;
		}

		// check if we have a full telegram in the buffer
		if(num_bytes_avail < telegram_bytes) {
			break;	// have to wait for more data
		}

		// check CRC
		uint16_t received_crc = 0;
		::memcpy(&received_crc, p_data + telegram_bytes - 2, 2);

		const uint16_t actual_crc = compute_crc(p_data, telegram_bytes - 2);
		if(actual_crc != received_crc)
		{
			discard_bytes(4);	// discard invalid bytes
			handle_debug_msg("actual_crc != received_crc");
			continue;
		}

		// read scan id
		uint32_t scan_id = 0;
		::memcpy(&scan_id, p_data + 10, 4);
		scan_id = ntohl(scan_id);

		// check data content type
		uint16_t data_content_type = 0;
		::memcpy(&data_content_type, p_data + 16, 2);

		if(data_content_type != 0xBBBB)
		{
			discard_bytes(4);	// discard invalid bytes
			handle_debug_msg("data_content_type != 0xBBBB");
			continue;
		}

		// check scan area
		uint16_t scan_area_code = 0;
		::memcpy(&scan_area_code, p_data + 18, 2);

		if(scan_area_code != 0x1111)
		{
			discard_bytes(4);	// discard invalid bytes
			handle_debug_msg("scan_area_code != 0x1111");
			continue;
		}

		const ssize_t num_points = telegram_size - 11;

		// read points
		std::vector<point_t> points(num_points);
		for(ssize_t i = 0; i < num_points; ++i)
		{
			uint16_t block = 0;
			::memcpy(&block, p_data + 20 + i * 2, 2);

			point_t& point = points[i];
			point.distance = float(block & 0x1FFF) * 0.01f;
			point.reflector = block & 0x2000;
			point.protective = block & 0x8000;
			point.warn_field = block & 0x4000;
		}

		// handle data
		handle_scan(points);

		// discard data
		discard_bytes(4 + telegram_bytes);
	}
}

void SickS300Receiver::discard_bytes(ssize_t num_bytes)
{
	if(num_bytes > m_offset) {
		throw std::logic_error("num_bytes > m_offset");
	}
	::memmove(m_buffer.data(), m_buffer.data() + num_bytes, m_offset - num_bytes);
	m_offset -= num_bytes;
}

uint16_t compute_crc(const uint8_t* p_data, ssize_t num_bytes)
{
	uint16_t crc_value = 0xFFFF;
	for(ssize_t i = 0; i < num_bytes; ++i) {
		crc_value = (crc_value << 8) ^ crc_table[(((uint8_t)(crc_value >> 8)) ^ p_data[i])];
	}
	return crc_value;
}

