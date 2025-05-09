// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Driver for CH347-JTAG interface V1.1                                  *
 *                                                                         *
 *   Copyright (C) 2022 by oidcat.                                         *
 *   Author: oidcatiot@163.com                                             *
 *                                                                         *
 *   Enhancements by EasyDevKits - info@easydevkits.com                    *
 *                                                                         *
 *   CH347 is a high-speed USB bus converter chip that provides UART, I2C  *
 *   and SPI synchronous serial ports and JTAG interface through USB bus.  *
 *                                                                         *
 *   The Jtag interface by CH347 can supports transmission frequency       *
 *   configuration up to 60MHz.                                            *
 *                                                                         *
 *   The USB2.0 to JTAG scheme based on CH347 can be used to build         *
 *   customized USB high-speed JTAG debugger and other products.           *
 *                                                                         *
 *            _____________                                                *
 *           |             |____JTAG/SWD (TDO,TDI,TMS,TCK,TRST)            *
 *      USB__|    CH347T   |                                               *
 *           |_____________|____UART(TXD1,RXD1,RTS1,CTS1,DTR1)             *
 *            ______|______                                                *
 *           |             |                                               *
 *           | 8 MHz XTAL  |                                               *
 *           |_____________|                                               *
 *                                                                         *
 *   This CH347 driver is only tested for the CH347T chip in mode 3.       *
 *   The CH347 datasheet mention another chip the CH347F which was not     *
 *   available for testing.                                                *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if IS_CYGWIN == 1
#include "windows.h"
#undef LOG_ERROR
#endif

// project specific includes
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <jtag/swd.h>
#include <helper/time_support.h>
#include <helper/replacements.h>
#include <helper/list.h>
#include <helper/binarybuffer.h>
#include "libusb_helper.h"

// system includes
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define TDI_H							BIT(4)
#define TDI_L							0
#define TMS_H							BIT(1)
#define TMS_L							0
#define TCK_H							BIT(0)
#define TCK_L							0
#define TRST_H							BIT(5)
#define TRST_L							0
#define LED_ON							1
#define LED_OFF							0
#define GPIO_CNT						8 // the CH347 has 8 GPIO's
#define USEABLE_GPIOS					0x78 /* mask which GPIO's are available in mode 3 of CH347T
												only GPIO3 (Pin11 / SCL), GPIO4 (Pin15 / ACT),
												GPIO5 (Pin9 / TRST) and GPIO6 (Pin2 / CTS1) are possible
												Tested only with CH347T not CH347F chip
												pin numbers are for CH347T */
/* For GPIO command: always set bits 7 and 6 for GPIO enable
	bits 5 and 4 for pin direction output bit 3 is the data bit */
#define GPIO_SET_L						(BIT(4) | BIT(5) | BIT(6) | BIT(7)) // value for setting a GPIO to low
#define GPIO_SET_H						(BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7)) // value for setting a GPIO to high

#define VENDOR_VERSION					0x5F // for getting the chip version

#define HW_TDO_BUF_SIZE					4096 /* maybe the hardware of the CH347 chip can
												capture only this amount of TDO bits */
#define LARGER_PACK_MAX_SIZE			51200 /* Don't send more than this amount of bytes via libusb in one packet.
												Also that is the limit for LARGER_PACK mode */
#define UCMDPKT_DATA_MAX_BYTES_USBHS	507   /* The data length contained in each command packet
												during USB high-speed operation */
#define USBC_PACKET_USBHS				512   // Maximum data length per packet at USB high speed
#define CH347_CMD_HEADER				3     // Protocol header length (1 byte command type + 2 bytes data length)
#define CH347_CMD_INIT_READ_LEN			1 // for JTAG_INIT/SWD_INIT we have only one data byte
#define MAX_BITS_PER_BIT_OP				248   /* No more bits are allowed per CH347_CMD_JTAG_BIT_OP
												 command; this should be dividable by 8 */

// Protocol transmission format: CMD (1 byte) + Length (2 bytes) + Data
#define CH347_CMD_INFO_RD				0xCA /* Parameter acquisition, used to obtain firmware version,
												JTAG interface related parameters, etc */
#define CH347_CMD_GPIO					0xCC // GPIO Command
#define CH347_CMD_JTAG_INIT				0xD0 // JTAG Interface initialization command
#define CH347_CMD_JTAG_BIT_OP			0xD1 // JTAG interface pin bit control command
#define CH347_CMD_JTAG_BIT_OP_RD		0xD2 // JTAG interface pin bit control and read commands
#define CH347_CMD_JTAG_DATA_SHIFT		0xD3 // JTAG interface data shift command
#define CH347_CMD_JTAG_DATA_SHIFT_RD	0xD4 // JTAG interface data shift and read command
// for a single command these amount of data can be read at max
#define CH347_SINGLE_CMD_MAX_READ		MAX(GPIO_CNT, CH347_CMD_INIT_READ_LEN)

/* for SWD */
#define CH347_CMD_SWD_INIT				0xE5 // SWD Interface Initialization Command
#define CH347_CMD_SWD					0xE8 // SWD Command group header
#define CH347_CMD_SWD_REG_W				0xA0 // SWD Interface write reg
#define CH347_CMD_SWD_SEQ_W				0xA1 // SWD Interface write spec seq
#define CH347_CMD_SWD_REG_R				0xA2 // SWD Interface read  reg
#define CH347_MAX_SEND_CMD				19 // max send cmd number
#define CH347_MAX_SEND_BUF				0X200
#define CH347_MAX_RECV_BUF				0X200
#define CH347_MAX_CMD_BUF				128

#define CH347_EPOUT						0x06u // the usb endpoint number for writing
#define CH347_EPIN						0x86u // the usb endpoint number for reading
#define CH347_MPHSI_INTERFACE			2 // the JTAG interface is number 2
#define USB_WRITE_TIMEOUT				500	// write timeout in milliseconds
#define USB_READ_TIMEOUT				500 // read timeout in milliseconds
#define BYTEWISE_MODE_VERSION			0x241 /* BCD version for devices that can use bytewise mode
												below this version only bitwise mode can be used */
#define DEFAULT_VENDOR_ID				0x1a86 // if no vendor id is set use this CH347 default
#define DEFAULT_PRODUCT_ID				0x55dd // if no product id is set use this CH347 default

// for STANDARD_PACK mode: these are the 6 possible speeds; values in kHz
static const int ch347_standard_pack_clock_speeds[] = {
	1875,	// 1.875 MHz (60000 : 32)
	3750,	// 3.75 MHz (60000 : 16)
	7500,	// 7.5 MHz (60000 : 8)
	15000,	// 15 MHz (60000 : 4)
	30000,	// 30 MHz (60000 : 2)
	60000	// 60 MHz
};

// for LARGER_PACK mode: these are the 8 possible speeds; values in Hertz
static const int ch347_larger_pack_clock_speeds[] = {
	469,	// 468.75 kHz (60000 : 128)
	938,	// 937.5 kHz (60000 : 64)
	1875,	// 1.875 MHz (60000 : 32)
	3750,	// 3.75 MHz (60000 : 16)
	7500,	// 7.5 MHz (60000 : 8)
	15000,	// 15 MHz (60000 : 4)
	30000,	// 30 MHz (60000 : 2)
	60000	// 60 MHz
};

/* STANDARD_PACK means that we can send only one USBC_PACKET_USBHS-sized USB packet
and then read data back. LARGER_PACK means, we can send packets as large as
LARGER_PACK_MAX_SIZE. libusb splits there large packets into smaller USB packets and
transmit the data. Then we read back the data in a bigger packet.
*/
enum pack_size {
	UNSET = -1,
	STANDARD_PACK = 0,
	LARGER_PACK = 1,
};

struct ch347_cmd {
	uint8_t type; // the command type
	uint8_t *write_data; // data bytes for write
	uint16_t write_data_len; // count of data bytes in the write_data buffer
	uint16_t read_len; // if >0 a read is needed after this command
	uint16_t tdo_bit_count; // how many TDO bits are needed to shift in by this read
	struct list_head queue; // for handling a queue (list)
};

struct ch347_scan {
	struct scan_field *fields; // array of scan_field's for data from the device
	int fields_len; // scan_fields array length
	struct list_head queue; // for handling a queue (list)
};

struct ch347_info {
	// Record the CH347 pin status
	int tms_pin;
	int tdi_pin;
	int tck_pin;
	int trst_pin;

	bool use_bitwise_mode; /* if true then we can't us the bytewise commands
								due to a bug of the chip; depends on BYTEWISE_MODE_VERSION */
	enum pack_size pack_size; // see: pack_size for explanation

	// a "scratchpad" where we record all bytes for one command
	uint8_t scratchpad_cmd_type; // command type
	uint8_t scratchpad[UCMDPKT_DATA_MAX_BYTES_USBHS]; // scratchpad buffer
	int scratchpad_idx; // current index in scratchpad

	// after a command is complete it will be stored for later processing
	struct list_head cmd_queue;
	// all data input scan fields are queued here
	struct list_head scan_queue;
	// read buffer for the single commands like CH347_CMD_GPIO and CH347_CMD_JTAG_INIT
	uint8_t single_read[CH347_SINGLE_CMD_MAX_READ];
	int singe_read_len; // data length in single_read
};

struct ch347_swd_io {
	uint8_t usb_cmd; // 0xA0, 0xA1, 0xA2
	uint8_t cmd;
	uint32_t *dst;
	uint32_t value;
	struct list_head list_entry;
};

struct ch347_swd_context {
	uint8_t send_buf[CH347_MAX_SEND_BUF];
	uint8_t recv_buf[CH347_MAX_RECV_BUF];
	int send_len;
	int recv_len;
	int need_recv_len;
	int queued_retval;
	int sent_cmd_count;
	struct list_head send_cmd_head;
	struct list_head free_cmd_head;
	struct ch347_swd_io ch347_cmd_buf[CH347_MAX_CMD_BUF];
};

static struct ch347_swd_context ch347_swd_context;
static bool swd_mode;
// porcodio
static uint16_t ch347_vids[] = {0x1a86, 0x1a86, 0x1a86, 0};
static uint16_t ch347_pids[] = {0x55dd, 0x55de, 0x55e7, 0};
static char *ch347_device_desc;
static uint8_t ch347_activity_led_gpio_pin = 0xFF;
static bool ch347_activity_led_active_high;
static struct ch347_info ch347;
static struct libusb_device_handle *ch347_handle;

/* there are "single" commands. These commands can't be chained together and
	need to be send as single command and need a read after write */
static inline bool ch347_is_single_cmd_type(uint8_t type)
{
	return type == CH347_CMD_GPIO || type == CH347_CMD_JTAG_INIT || type == CH347_CMD_SWD_INIT;
}

/**
 * @brief writes data to the CH347 via libusb driver
 *
 * @param data Point to the data buffer
 * @param length Data length in and out
 * @return ERROR_OK at success
 */
static int ch347_write_data(uint8_t *data, int *length)
{
	int write_len = *length;
	int i = 0;
	int transferred = 0;

	while (true) {
		int retval = jtag_libusb_bulk_write(ch347_handle, CH347_EPOUT, (char *)&data[i],
			write_len, USB_WRITE_TIMEOUT, &transferred);
		if (retval != ERROR_OK) {
			LOG_ERROR("CH347 write fail");
			*length = 0;
			return retval;
		}
		i += transferred;
		if (i >= *length)
			break;
		write_len = *length - i;
	}

	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
		char *str = buf_to_hex_str(data, i * 8);
		LOG_DEBUG_IO("size=%d, buf=[%s]", i, str);
		free(str);
	}

	*length = i;
	return ERROR_OK;
}

/**
 * @brief reads data from the CH347 via libusb driver
 *
 * @param data Point to the data buffer
 * @param length Data length in and out
 * @return ERROR_OK at success
 */
static int ch347_read_data(uint8_t *data, int *length)
{
	int read_len = *length;
	int i = 0;
	int transferred = 0;

	while (true) {
		int retval = jtag_libusb_bulk_read(ch347_handle, CH347_EPIN, (char *)&data[i],
			read_len, USB_READ_TIMEOUT, &transferred);
		if (retval != ERROR_OK) {
			LOG_ERROR("CH347 read fail");
			*length = 0;
			return retval;
		}

		i += transferred;
		if (i >= *length)
			break;
		read_len = *length - i;
	}

	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
		char *str = buf_to_hex_str(data, i * 8);
		LOG_DEBUG_IO("size=%d, buf=[%s]", i, str);
		free(str);
	}

	*length = i;
	return ERROR_OK;
}

/**
 * @brief calculates the amount of bits and bytes that should be read
 * for this command
 *
 * @param cmd command for the calculation
 */
static void ch347_cmd_calc_reads(struct ch347_cmd *cmd)
{
	cmd->read_len = 0;
	cmd->tdo_bit_count = 0;

	switch (cmd->type) {
	case CH347_CMD_GPIO:
		// for GPIO we need to read back the same amount of data that we had send
		cmd->read_len = cmd->write_data_len;
		break;
	case CH347_CMD_JTAG_INIT:
	case CH347_CMD_SWD_INIT:
		// for JTAG_INIT/SWD_INIT the amount is fixed
		cmd->read_len = CH347_CMD_INIT_READ_LEN;
		break;
	case CH347_CMD_JTAG_BIT_OP_RD:
		// for bit operations we need to count the TCK high edges
		for (int i = 0; i < cmd->write_data_len; i++) {
			if ((cmd->write_data[i] & TCK_H) == TCK_H) {
				cmd->read_len++;
				cmd->tdo_bit_count++;
			}
		}
		break;
	case CH347_CMD_JTAG_DATA_SHIFT_RD:
		// for byte operations: need to read one byte back for each data byte
		cmd->read_len = cmd->write_data_len;
		// we occupy 8 bits per byte in the TDO hardware buffer
		cmd->tdo_bit_count = cmd->read_len * 8;
		break;
	}
}

/**
 * @brief copy the scratchpad content into a new command in the command queue
 *
 * @param scan_fields array of scan_field's for data from the device
 * @param scan_fields_len array length
 */
static void ch347_cmd_from_scratchpad(void)
{
	// nothing to do if no bytes are recorded
	if (!ch347.scratchpad_idx)
		return;

	// malloc for the command and data bytes
	struct ch347_cmd *cmd = malloc(sizeof(struct ch347_cmd));
	if (cmd)
		cmd->write_data = malloc(ch347.scratchpad_idx);
	if (!cmd || !cmd->write_data) {
		LOG_ERROR("malloc failed");
		free(cmd);
		return;
	}

	// copy data, calculate the reads and add to the command queue
	cmd->type = ch347.scratchpad_cmd_type;
	cmd->write_data_len = ch347.scratchpad_idx;
	memcpy(cmd->write_data, ch347.scratchpad, ch347.scratchpad_idx);
	ch347_cmd_calc_reads(cmd);
	list_add_tail(&cmd->queue, &ch347.cmd_queue);

	// cleanup the scratchpad for the next command
	ch347.scratchpad_cmd_type = 0;
	ch347.scratchpad_idx = 0;
}

/**
 * @brief Reads data back from CH347 and decode it byte- and bitwise into the buffer
 *
 * @param decoded_buf Point to a buffer to place the data to be decoded; need to be sized
 * to the decoded size length; not the raw_read_len
 * @param decoded_buf_len length of the decoded_buf
 * @param raw_read_len Data length in bytes that should be read via libusb; the decoded length can be shorther
 */
static void ch347_read_scan(uint8_t *decoded_buf, int decoded_buf_len, int raw_read_len)
{
	int read_len = raw_read_len;
	uint8_t *read_buf = malloc(read_len);
	if (!read_buf) {
		LOG_ERROR("malloc failed");
		return;
	}

	if (ch347_read_data(read_buf, &read_len) != ERROR_OK) {
		free(read_buf);
		return;
	}

	int rd_idx = 0;
	int decoded_buf_idx = 0;

	while (rd_idx < read_len) {
		unsigned int type = read_buf[rd_idx++];
		uint16_t data_len = le_to_h_u16(&read_buf[rd_idx]);
		rd_idx += 2;
		if (decoded_buf_idx > decoded_buf_len) {
			LOG_ERROR("CH347 decoded_buf too small");
			break;
		}

		// nothing to decode? Only read to make the CH347 happy!
		if (!decoded_buf) {
			rd_idx += data_len;
			continue;
		}

		switch (type) {
		case CH347_CMD_GPIO:
		case CH347_CMD_JTAG_INIT:
		case CH347_CMD_SWD_INIT:
		case CH347_CMD_JTAG_DATA_SHIFT_RD:
			// for all bytewise commands: copy the data bytes
			memcpy(&decoded_buf[decoded_buf_idx], &read_buf[rd_idx], data_len);
			decoded_buf_idx += data_len;
			rd_idx += data_len;
			break;
		case CH347_CMD_JTAG_BIT_OP_RD:
			// for CH347_CMD_JTAG_BIT_OP_RD we need to copy bit by bit
			for (int i = 0; i < data_len; i++) {
				if (read_buf[rd_idx + i] & BIT(0))
					decoded_buf[decoded_buf_idx + i / 8] |= BIT(i % 8);
				else
					decoded_buf[decoded_buf_idx + i / 8] &= ~(BIT(i % 8));
			}
			rd_idx += data_len;
			decoded_buf_idx += DIV_ROUND_UP(data_len, 8);
			break;
		default:
			LOG_ERROR("CH347 read command fail");
			free(read_buf);
			return;
		}
	}

	free(read_buf);
}

/**
 * @brief Used to put the data from the decoded buffer into the scan command fields
 *
 * @param decoded_buf Point to a buffer for the decoded data
 * @param decoded_buf_len length of the decoded_buf
 */
static void ch347_scan_data_to_fields(uint8_t *decoded_buf, int decoded_buf_len)
{
	int byte_offset = 0;
	struct ch347_scan *scan;
	struct ch347_scan *tmp;
	int bit_offset = 0;
	list_for_each_entry_safe(scan, tmp, &ch347.scan_queue, queue) {
		for (int i = 0; i < scan->fields_len; i++) {
			int num_bits = scan->fields[i].num_bits;
			LOG_DEBUG("fields[%d].in_value[%d], read from bit offset: %d", i, num_bits, bit_offset);
			// only if we need the value
			if (scan->fields[i].in_value) {
				uint8_t *capture_buf = malloc(DIV_ROUND_UP(num_bits, 8));
				if (!capture_buf) {
					LOG_ERROR("malloc failed");
					return;
				}
				uint8_t *captured = buf_set_buf(decoded_buf, bit_offset, capture_buf, 0, num_bits);

				if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
					char *str = buf_to_hex_str(captured, num_bits);
					LOG_DEBUG_IO("size=%d, buf=[%s]", num_bits, str);
					free(str);
				}

				buf_cpy(captured, scan->fields[i].in_value, num_bits);
				free(capture_buf);
			} else {
				if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO))
					LOG_DEBUG_IO("field skipped");
			}
			bit_offset += num_bits;
		}
		list_del(&scan->queue);
		free(scan);
		/* after one round of scan field processing the
			next data bits are read from the next data byte
			=> round up and calculate the next start bit */
		byte_offset = DIV_ROUND_UP(bit_offset, 8);
		bit_offset = byte_offset * 8;
	}

	// if not all bytes are transferred: put the rest into single_read buffer
	if (byte_offset < decoded_buf_len) {
		ch347.singe_read_len = decoded_buf_len - byte_offset;
		LOG_DEBUG("single read of %d bytes", ch347.singe_read_len);
		if (ch347.singe_read_len > CH347_SINGLE_CMD_MAX_READ) {
			LOG_ERROR("Can't read more than %d bytes for a single command!", CH347_SINGLE_CMD_MAX_READ);
			ch347.singe_read_len = CH347_SINGLE_CMD_MAX_READ;
		}
		memcpy(ch347.single_read, &decoded_buf[byte_offset], ch347.singe_read_len);
	}
}

/**
 * @brief Sends the write buffer via libusb
 * and if LARGER_PACK mode is active read also data back
 */
static void ch347_cmd_transmit_queue(void)
{
	// queue last command
	ch347_cmd_from_scratchpad();

	if (list_empty(&ch347.cmd_queue))
		return;

	// calculate the needed buffer length for all decoded bytes
	struct ch347_cmd *cmd;
	int decoded_buf_len = 0;
	list_for_each_entry(cmd, &ch347.cmd_queue, queue)
		if (cmd->read_len > 0)
			decoded_buf_len += ch347_is_single_cmd_type(cmd->type) ?
				cmd->read_len : DIV_ROUND_UP(cmd->tdo_bit_count, 8);

	// create the buffer for all decoded bytes
	uint8_t *decoded_buf = NULL;
	int decoded_buf_idx = 0;
	if (decoded_buf_len > 0) {
		decoded_buf = malloc(decoded_buf_len);
		if (!decoded_buf) {
			LOG_ERROR("malloc failed");
			return;
		}
	}

	while (!list_empty(&ch347.cmd_queue)) {
		struct ch347_cmd *last_cmd = NULL;
		int total_len = 0;
		int total_tdo_count = 0;
		int bytes_to_write = 0;
		// in STANDARD_PACK or bitwise mode we can send only one USBC_PACKET_USBHS sized package
		int max_len = ch347.pack_size == STANDARD_PACK || ch347.use_bitwise_mode ?
			USBC_PACKET_USBHS : LARGER_PACK_MAX_SIZE;
		list_for_each_entry(cmd, &ch347.cmd_queue, queue) {
			total_len += CH347_CMD_HEADER + cmd->write_data_len;
			total_tdo_count += cmd->tdo_bit_count;
			// don't exceed max length or max TDO bit count
			if (total_len >= max_len || total_tdo_count >= HW_TDO_BUF_SIZE)
				break;
			// remember the last cmd to send and bytes to send
			last_cmd = cmd;
			bytes_to_write = total_len;
		}

		// sanity checks
		if (!last_cmd || bytes_to_write == 0) {
			LOG_ERROR("Nothing to send!");
			free(decoded_buf);
			return;
		}

		// create the write buffer
		uint8_t *write_buf = malloc(bytes_to_write);
		if (!write_buf) {
			LOG_ERROR("malloc failed");
			free(decoded_buf);
			return;
		}

		int idx = 0;
		int bytes_to_read = 0;
		int current_decoded_buf_len = 0;
		struct ch347_cmd *tmp;

		list_for_each_entry_safe(cmd, tmp, &ch347.cmd_queue, queue) {
			// copy command to buffer
			write_buf[idx++] = cmd->type;
			h_u16_to_le(&write_buf[idx], cmd->write_data_len);
			idx += 2;
			memcpy(&write_buf[idx], cmd->write_data, cmd->write_data_len);
			idx += cmd->write_data_len;
			// need to read something back?
			if (cmd->read_len > 0) {
				bytes_to_read += CH347_CMD_HEADER + cmd->read_len;
				current_decoded_buf_len += ch347_is_single_cmd_type(cmd->type) ?
					cmd->read_len : DIV_ROUND_UP(cmd->tdo_bit_count, 8);
			}

			// cmd data no longer needed
			list_del(&cmd->queue);
			free(cmd->write_data);
			free(cmd);

			if (cmd == last_cmd)
				break;
		}

		// write data to device
		if (ch347_write_data(write_buf, &idx) != ERROR_OK) {
			free(write_buf);
			free(decoded_buf);
			return;
		}
		free(write_buf);

		if (!bytes_to_read)
			continue;

		// Need only to execute a read without decoding the data to make the CH347 happy?
		if (!current_decoded_buf_len) {
			// read but don't decode anything
			ch347_read_scan(NULL, 0, bytes_to_read);
		} else {
			ch347_read_scan(&decoded_buf[decoded_buf_idx], current_decoded_buf_len, bytes_to_read);
			decoded_buf_idx += current_decoded_buf_len;
		}
	}

	// something decoded from the data read back from CH347?
	if (decoded_buf) {
		// put the decoded data into the scan fields or single_read buffer
		ch347_scan_data_to_fields(decoded_buf, decoded_buf_len);
		free(decoded_buf);
	}
}

/**
 * @brief starts the next command in the scratchpad. If it's the same command type
 * it can concat the data bytes. no need to make a new command for this case
 *
 * @param type command type
 */
static void ch347_cmd_start_next(uint8_t type)
{
	// different command type or non chainable command? (GPIO commands can't be concat)
	uint8_t prev_type = ch347.scratchpad_cmd_type;
	if (prev_type != type || ch347_is_single_cmd_type(type)) {
		// something written in the scratchpad? => store it as command
		if (prev_type != 0 && ch347.scratchpad_idx > 0) {
			ch347_cmd_from_scratchpad();
			/* if the last queued command is not chainable we should send it immediately
				because e.g. the GPIO command can't be combined with any other command */
			if (ch347_is_single_cmd_type(prev_type))
				ch347_cmd_transmit_queue();
		}

		/* before we can send non chainable command ("single" like GPIO command) we should send all
			other commands because we can't send it together with other commands */
		if (ch347_is_single_cmd_type(type))
			ch347_cmd_transmit_queue();

		// store the next command type
		ch347.scratchpad_cmd_type = type;
	}
}

/**
 * @brief queue the scan fields into the scan queue
 *
 * @param scan_fields array of scan field's for data from the device
 * @param scan_fields_len array length
 */
static void ch347_scan_queue_fields(struct scan_field *scan_fields, int scan_fields_len)
{
	// malloc for the scan struct
	struct ch347_scan *scan = malloc(sizeof(struct ch347_scan));
	if (!scan) {
		LOG_ERROR("malloc failed");
		return;
	}
	scan->fields = scan_fields;
	scan->fields_len = scan_fields_len;
	list_add_tail(&scan->queue, &ch347.scan_queue);
}

/**
 * @brief Function executes the single command and deliver one byte from the buffer
 * that's read back from USB
 *
 * @param read_buf_idx index of the byte that should be returned
 * @return byte at index or if index is out of range the first byte
 */
static uint8_t ch347_single_read_get_byte(int read_buf_idx)
{
	ch347_cmd_transmit_queue();
	if (read_buf_idx > CH347_SINGLE_CMD_MAX_READ || read_buf_idx < 0) {
		LOG_ERROR("read_buf_idx out of range");
		read_buf_idx = 0;
	}
	return ch347.single_read[read_buf_idx];
}

/**
 * @brief checks if the scratchpad is full. If it's full the function creates
 * a command from it and puts it into the command queue
 */
static void ch347_scratchpad_check_full(void)
{
	// if full create a new command in the queue
	if (ch347.scratchpad_idx == UCMDPKT_DATA_MAX_BYTES_USBHS) {
		uint8_t type = ch347.scratchpad_cmd_type;
		ch347_cmd_from_scratchpad();
		ch347.scratchpad_cmd_type = type;
	}
}

/**
 * @brief adds one byte to the scratchpad
 * if scratchpad is full after this byte the command will be created from the
 * scratchpad and the scratchpad is cleared for the next command
 *
 * @param byte add this byte
 */
static void ch347_scratchpad_add_byte(uint8_t byte)
{
	if (ch347.scratchpad_cmd_type == 0) {
		LOG_ERROR("call ch347_next_cmd first!");
		return;
	}
	ch347.scratchpad[ch347.scratchpad_idx++] = byte;
	ch347_scratchpad_check_full();
}

/**
 * @brief adds the output pin byte to the scratchpad
 * if scratchpad is full after this byte the command will be created from the
 * scratchpad and the scratchpad is cleared for the next command
 */
static void ch347_scratchpad_add_pin_byte(void)
{
	ch347_scratchpad_add_byte(ch347.tms_pin | ch347.tdi_pin | ch347.tck_pin | ch347.trst_pin);
}

/**
 * @brief adds bytes from a buffer to the scratchpad
 * if scratchpad is full after this byte the command will be created from the
 * scratchpad and the scratchpad is cleared for the next command
 *
 * @param bytes add this bytes
 * @param count byte count
*/
static void ch347_scratchpad_add_bytes(uint8_t *bytes, int count)
{
	if (ch347.scratchpad_cmd_type == 0) {
		LOG_ERROR("call ch347_next_cmd first!");
		return;
	}

	int remaining = count;
	int bytes_idx = 0;
	while (remaining > 0) {
		int bytes_to_store = ch347.scratchpad_idx + remaining <= UCMDPKT_DATA_MAX_BYTES_USBHS ?
			remaining :	UCMDPKT_DATA_MAX_BYTES_USBHS - ch347.scratchpad_idx;

		if (bytes)
			memcpy(&ch347.scratchpad[ch347.scratchpad_idx], &bytes[bytes_idx], bytes_to_store);
		else
			memset(&ch347.scratchpad[ch347.scratchpad_idx], 0, bytes_to_store);

		ch347.scratchpad_idx += bytes_to_store;
		bytes_idx += bytes_to_store;
		remaining -= bytes_to_store;
		ch347_scratchpad_check_full();
	}
}

/**
 * @brief Function used to change the TMS value at the
 * rising edge of TCK to switch its TAP state
 *
 * @param tms TMS value to be changed; true = output TMS high; false = output TMS low
 */
static void ch347_scratchpad_add_clock_tms(bool tms)
{
	ch347.tms_pin = tms ? TMS_H : TMS_L;
	ch347.tck_pin = TCK_L;
	ch347_scratchpad_add_pin_byte();
	ch347.tck_pin = TCK_H;
	ch347_scratchpad_add_pin_byte();
}

/**
 * @brief Function adds a certain amount of TCK pulses without changing the TMS pin
 *
 * @param count Amount of L/H TCK pulses to add
 */
static void ch347_scratchpad_add_stableclocks(int count)
{
	bool tms = ch347.tms_pin == TMS_H;
	for (int i = 0; i < count; i++)
		ch347_scratchpad_add_clock_tms(tms);
}

/**
 * @brief Function to ensure that the clock is in a low state
 */
static void ch347_scratchpad_add_idle_clock(void)
{
	ch347.tck_pin = TCK_L;
	ch347_scratchpad_add_pin_byte();
}

/**
 * @brief Function that performs state switching by changing the value of TMS
 *
 * @param tms_value The TMS values that need to be switched form one byte of data in the switching order
 * @param step The number of bit values that need to be read from the tms_value value
 * @param skip Count from the skip bit of tms_value to step
 */
static void ch347_scratchpad_add_tms_change(const uint8_t *tms_value, int step, int skip)
{
	LOG_DEBUG_IO("TMS Value: %02x..., step = %d, skip = %d", tms_value[0], step, skip);

	ch347_cmd_start_next(CH347_CMD_JTAG_BIT_OP);
	for (int i = skip; i < step; i++)
		ch347_scratchpad_add_clock_tms((tms_value[i / 8] >> (i % 8)) & BIT(0));
	ch347_scratchpad_add_idle_clock();
}


/**
 * @brief Obtain the current Tap status and switch to the status TMS value passed down by cmd
 *
 * @param cmd Upper layer transfer command parameters
 * @return ERROR_OK at success; ERROR_JTAG_TRANSITION_INVALID if no transition is possible
 */
static int ch347_scratchpad_add_move_path(struct pathmove_command *cmd)
{
	LOG_DEBUG_IO("num_states=%d, last_state=%d", cmd->num_states, cmd->path[cmd->num_states - 1]);

	ch347_cmd_start_next(CH347_CMD_JTAG_BIT_OP);
	for (int i = 0; i < cmd->num_states; i++) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[i]) {
			ch347_scratchpad_add_clock_tms(0);
		} else if (tap_state_transition(tap_get_state(), true) == cmd->path[i]) {
			ch347_scratchpad_add_clock_tms(1);
		} else {
			LOG_ERROR("No transition possible!");
			return ERROR_JTAG_TRANSITION_INVALID;
		}
		tap_set_state(cmd->path[i]);
	}
	ch347_scratchpad_add_idle_clock();
	return ERROR_OK;
}

/**
 * @brief Toggle the tap state to the target state
 *
 * @param state Pre switch target path
 * @param skip Number of digits to skip
 */
static void ch347_scratchpad_add_move_state(tap_state_t state, int skip)
{
	uint8_t tms_scan;
	int tms_len;

	LOG_DEBUG_IO("from %s to %s", tap_state_name(tap_get_state()), tap_state_name(state));
	// don't do anything if we are already in the right state; but do execute always the TAP_RESET
	if (tap_get_state() == state && state != TAP_RESET)
		return;
	tms_scan = tap_get_tms_path(tap_get_state(), state);
	tms_len = tap_get_tms_path_len(tap_get_state(), state);
	ch347_scratchpad_add_tms_change(&tms_scan, tms_len, skip);
	tap_set_state(state);
}

/**
 * @brief CH347 Batch read/write function
 *
 * @param cmd The scan command
 * @param bits Read and write data this time
 * @param bits_len Incoming data length in bits
 * @param scan The transmission method of incoming data to determine whether to perform data reading
 */
static void ch347_scratchpad_add_write_read(struct scan_command *cmd, uint8_t *bits, int bits_len, enum scan_type scan)
{
	// the bits and bytes to transfer
	int byte_count = bits_len / 8;
	int bit_count = bits_len % 8;

	// only bytes are not possible because we need to set TMS high for the last bit
	if (byte_count > 0 && bit_count == 0) {
		// make one byte to eight bits
		byte_count--;
		bit_count = 8;
	}

	// in bitwise mode only bits are allowed
	if (ch347.use_bitwise_mode) {
		byte_count = 0;
		bit_count = bits_len;
	}

	bool is_read = (scan == SCAN_IN || scan == SCAN_IO);

	// if we need to send bytes
	if (byte_count > 0) {
		// start the next cmd and copy the data out bytes to it
		ch347_cmd_start_next(is_read ? CH347_CMD_JTAG_DATA_SHIFT_RD : CH347_CMD_JTAG_DATA_SHIFT);
		if (bits)
			ch347_scratchpad_add_bytes(bits, byte_count);
		else
			ch347_scratchpad_add_bytes(NULL, byte_count);
	}

	// bits are always need to send; no possibility to not send bits
	ch347_cmd_start_next(is_read ? CH347_CMD_JTAG_BIT_OP_RD : CH347_CMD_JTAG_BIT_OP);

	ch347.tms_pin = TMS_L;
	ch347.tdi_pin = TDI_L;

	for (int i = 0; i < bit_count; i++) {
		if (bits)
			ch347.tdi_pin = ((bits[byte_count + i / 8] >> i % 8) & BIT(0)) ? TDI_H : TDI_L;

		// for the last bit set TMS high to exit the shift state
		if (i + 1 == bit_count)
			ch347.tms_pin = TMS_H;

		ch347.tck_pin = TCK_L;
		ch347_scratchpad_add_pin_byte();
		ch347.tck_pin = TCK_H;
		ch347_scratchpad_add_pin_byte();
		/* cut the package after each MAX_BITS_PER_BIT_OP bits because it
			needs a dividable by 8 bits package */
		if (i > 0 && (i + 1) % MAX_BITS_PER_BIT_OP == 0) {
			ch347_cmd_from_scratchpad();
			ch347_cmd_start_next(is_read ? CH347_CMD_JTAG_BIT_OP_RD : CH347_CMD_JTAG_BIT_OP);
		}
	}
	// one TCK_L after the last bit
	ch347_scratchpad_add_idle_clock();

	// if read is involed we need to queue the scan fields
	if (is_read)
		ch347_scan_queue_fields(cmd->fields, cmd->num_fields);
}

/**
 * @brief Toggle the Tap state to run test/idle
 *
 * @param cycles how many clock cycles for output
 * @param state JTAG end state
 */
static void ch347_scratchpad_add_run_test(int cycles, tap_state_t state)
{
	LOG_DEBUG_IO("cycles=%d, end_state=%d", cycles, state);
	if (tap_get_state() != TAP_IDLE)
		ch347_scratchpad_add_move_state(TAP_IDLE, 0);

	ch347_scratchpad_add_stableclocks(cycles);
	ch347_scratchpad_add_move_state(state, 0);
}

/**
 * @brief Switch to SHIFT-DR or SHIFT-IR status for scanning
 *
 * @param cmd Upper layer transfer command parameters
 * @return Always ERROR_OK
 */
static int ch347_scratchpad_add_scan(struct scan_command *cmd)
{
	static const char *const type2str[] = {"", "SCAN_IN", "SCAN_OUT", "SCAN_IO"};

	enum scan_type type = jtag_scan_type(cmd);
	uint8_t *buf = NULL;
	int scan_bits = jtag_build_buffer(cmd, &buf);

	// add a move to IRSHIFT or DRSHIFT state
	if (cmd->ir_scan)
		ch347_scratchpad_add_move_state(TAP_IRSHIFT, 0);
	else
		ch347_scratchpad_add_move_state(TAP_DRSHIFT, 0);

	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
		char *log_buf = buf_to_hex_str(buf, scan_bits);
		LOG_DEBUG_IO("scan=%s, type=%s, bits=%d, buf=[%s], end_state=%d",
			 cmd->ir_scan ? "IRSCAN" : "DRSCAN",
			 type2str[type],
			 scan_bits, log_buf, cmd->end_state);
		free(log_buf);
	}

	ch347_scratchpad_add_write_read(cmd, buf, scan_bits, type);
	free(buf);

	// add a move to the final state
	ch347_scratchpad_add_move_state(cmd->end_state, 1);

	return ERROR_OK;
}

/**
 * @brief Sets a GPIO bit
 *
 * @param gpio GPIO bit number 0-7
 * @param data true for high; false for low
 */
static void ch347_gpio_set(int gpio, bool data)
{
	ch347_cmd_start_next(CH347_CMD_GPIO);
	uint8_t gpios[GPIO_CNT];
	memset(gpios, 0, GPIO_CNT);
	/* always set bits 7 and 6 for GPIO enable
		bits 5 and 4 for pin direction output
		bit 3 is the data bit */
	gpios[gpio] = data == 0 ? GPIO_SET_L : GPIO_SET_H;
	ch347_scratchpad_add_bytes(gpios, GPIO_CNT);
	// check in the read if the bit is set/cleared correctly
	if ((ch347_single_read_get_byte(gpio) & BIT(6)) >> 6 != data)
		LOG_ERROR("Output not set.");
}

/**
 * @brief Turn the activity LED on or off
 *
 * @param led_state LED_ON or LED_OFF
 */
static void ch347_activity_led_set(int led_state)
{
	if (ch347_activity_led_gpio_pin != 0xFF)
		ch347_gpio_set(ch347_activity_led_gpio_pin, ch347_activity_led_active_high ? led_state : 1 - led_state);
}

/**
 * @brief Sets the TRST pin
 *
 * @param status Pin status: true = high; false = low
 * @return ERROR_OK at success; ERROR_FAIL otherwise
 */
static int ch347_trst_set(bool status)
{
	ch347_cmd_start_next(CH347_CMD_JTAG_BIT_OP);
	ch347.trst_pin = status ? TRST_H : TRST_L;
	ch347_scratchpad_add_pin_byte();
	ch347_cmd_transmit_queue();
	return ERROR_OK;
}

/**
 * @brief Control (assert/deassert) the signals SRST and TRST on the interface.
 *
 * @param trst 1 to assert SRST, 0 to deassert SRST.
 * @param srst 1 to assert TRST, 0 to deassert TRST.
 * @return Always ERROR_FAIL for asserting via SRST and TRST in SWDW mode.
 * ERROR_OK for assert/deassert in JTAG mode for TRST
 */
static int ch347_reset_assert(int trst, int srst)
{
	LOG_DEBUG_IO("reset trst: %i srst %i", trst, srst);
	if (srst) {
		LOG_INFO("Asserting SRST not supported!");
		return ERROR_FAIL;
	}

	if (swd_mode) {
		if (trst)
			LOG_INFO("Asserting TRST not supported in SWD mode!");
		return ERROR_FAIL;
	}

	ch347_cmd_start_next(CH347_CMD_JTAG_BIT_OP);
	ch347.trst_pin = trst ? TRST_L : TRST_H;
	ch347_scratchpad_add_pin_byte();
	ch347_scratchpad_add_idle_clock();
	ch347_cmd_transmit_queue();
	return ERROR_OK;
}

/**
 * @brief Flushes the command buffer and sleeps for a specific timspan
 *
 * @param us Sleep time in microseconds
 */
static void ch347_sleep(int us)
{
	LOG_DEBUG_IO("us=%d", us);
	ch347_cmd_transmit_queue();
	jtag_sleep(us);
}

/**
 * @brief Executes the command quene
 *
 * @return Success returns ERROR_OK
 */
static int ch347_execute_queue(void)
{
	struct jtag_command *cmd;
	int retval = ERROR_OK;

	ch347_activity_led_set(LED_ON);

	for (cmd = jtag_command_queue; retval == ERROR_OK && cmd;
		 cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RUNTEST:
			ch347_scratchpad_add_run_test(cmd->cmd.runtest->num_cycles,
					  cmd->cmd.runtest->end_state);
			break;
		case JTAG_STABLECLOCKS:
			ch347_scratchpad_add_stableclocks(cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_TLR_RESET:
			ch347_scratchpad_add_move_state(cmd->cmd.statemove->end_state, 0);
			break;
		case JTAG_PATHMOVE:
			retval = ch347_scratchpad_add_move_path(cmd->cmd.pathmove);
			break;
		case JTAG_TMS:
			ch347_scratchpad_add_tms_change(cmd->cmd.tms->bits, cmd->cmd.tms->num_bits, 0);
			break;
		case JTAG_SLEEP:
			ch347_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			retval = ch347_scratchpad_add_scan(cmd->cmd.scan);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type 0x%X", cmd->type);
			retval = ERROR_FAIL;
			break;
		}
	}

	ch347_activity_led_set(LED_OFF);
	ch347_cmd_transmit_queue();
	return retval;
}

/**
 * @brief opens the CH347 device via libusb driver
 *
 * @return ERROR_OK on success
 */
static int ch347_open_device(void)
{
	int retval = jtag_libusb_open(ch347_vids, ch347_pids, ch347_device_desc, &ch347_handle, NULL);
	if (retval != ERROR_OK) {
		LOG_ERROR("CH347 not found: vid=%04x, pid=%04x",  ch347_vids[0], ch347_pids[0]);
		return retval;
	}

	struct libusb_device_descriptor ch347_device_descriptor;
	libusb_device *device = libusb_get_device(ch347_handle);
	if (!device) {
		LOG_ERROR("CH347 error calling libusb_get_device");
		jtag_libusb_close(ch347_handle);
		return ERROR_FAIL;
	}

	retval = libusb_get_device_descriptor(device, &ch347_device_descriptor);
	if (retval != ERROR_OK) {
		LOG_ERROR("CH347 error getting device descriptor: %s", libusb_error_name(retval));
		jtag_libusb_close(ch347_handle);
		return retval;
	}

	//Doppio Porcodio
	/*retval = libusb_claim_interface(ch347_handle, CH347_MPHSI_INTERFACE);
	if (retval != ERROR_OK) {
		LOG_ERROR("CH347 unable to claim interface: %s", libusb_error_name(retval));
		jtag_libusb_close(ch347_handle);
		return retval;
	}
	*/
	/* jtag_libusb_open() already detached any kernel driver and
  	* claimed the right interface. Claiming it again would return
  	* LIBUSB_ERROR_BUSY. */

	char firmware_version;
	retval = jtag_libusb_control_transfer(ch347_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		VENDOR_VERSION, 0, 0, &firmware_version, sizeof(firmware_version),
		USB_WRITE_TIMEOUT, NULL);
	if (retval != ERROR_OK) {
		LOG_ERROR("CH347 unable to get firmware version");
		jtag_libusb_close(ch347_handle);
		return retval;
	}

	char manufacturer[256 + 1];
	if (libusb_get_string_descriptor_ascii(ch347_handle, ch347_device_descriptor.iManufacturer,
		(unsigned char *)manufacturer, sizeof(manufacturer) - 1) < 0) {
		strcpy(manufacturer, "(unknown)");
	}
	char product[256 + 1];
	if (libusb_get_string_descriptor_ascii(ch347_handle, ch347_device_descriptor.iProduct,
		(unsigned char *)product, sizeof(product) - 1) < 0) {
		strcpy(product, "(unknown)");
	}
	char serial_number[256 + 1];
	if (libusb_get_string_descriptor_ascii(ch347_handle, ch347_device_descriptor.iSerialNumber,
		(unsigned char *)serial_number, sizeof(serial_number) - 1) < 0) {
		strcpy(serial_number, "(unknown)");
	}

	LOG_INFO("CH347 %s from vendor %s with serial number %s found. (Chip version=%X.%2X, Firmware=0x%02X)",
		product,
		manufacturer,
		serial_number,
		(ch347_device_descriptor.bcdDevice >> 8) & 0xFF,
		ch347_device_descriptor.bcdDevice & 0xFF,
		firmware_version);

	if (ch347_device_descriptor.bcdDevice < BYTEWISE_MODE_VERSION) {
		LOG_INFO("CH347 old version of the chip, JTAG only working in bitwise mode. For bytewise mode at least version %X.%X is needed.",
			(BYTEWISE_MODE_VERSION >> 8) & 0xFF,
			BYTEWISE_MODE_VERSION & 0xFF);
		ch347.use_bitwise_mode = true;
	} else {
		ch347.use_bitwise_mode = false;
	}
	return ERROR_OK;
}

/**
 * @brief CH347 Device Release Function
 *
 * @return always returns ERROR_OK
 */
static int ch347_quit(void)
{
	// on close set the LED on, because the state without JTAG is on
	ch347_activity_led_set(LED_ON);
	ch347_cmd_transmit_queue();
	jtag_libusb_close(ch347_handle);
	LOG_DEBUG_IO("CH347 close");
	return ERROR_OK;
}

/**
 * @brief Sends the CH347_CMD_JTAG_INIT (D0) command to get the jtag
 * interface initialized with the speed index
 *
 * @param clock_index Clock index
 * @return Depends on clock_index
 * If clock_index is 9: true if the device supports STANDARD_PACK mode;
 * false if the device supports LARGER_PACK mode
 * If clock_index is not 9: true on success; false on error
 */
static bool ch347_adapter_speed_set(uint8_t clock_index)
{
	ch347_cmd_start_next(CH347_CMD_JTAG_INIT);
	ch347_scratchpad_add_byte(0);
	ch347_scratchpad_add_byte(clock_index);
	for (int i = 0; i < 4; i++)
		ch347_scratchpad_add_pin_byte();
	return ch347_single_read_get_byte(0) == 0;
}

/**
 * @brief Initializes the jtag interface and set CH347 TCK frequency
 *
 * @param speed_index speed index for JTAG_INIT command
 * @return Success returns ERROR_OK，failed returns ERROR_FAIL
 */
static int ch347_speed_set(int speed_index)
{
	if (swd_mode)
		return ERROR_OK;

	if (!ch347_adapter_speed_set(speed_index)) {
		LOG_ERROR("Couldn't set CH347 TCK speed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/**
 * @brief returns the speed in kHz by the give speed index
 *
 * @param speed_idx CH347 speed index
 * @param khz Output of the speed in kHz
 * @return Always ERROR_OK
 */
static int ch347_speed_get(int speed_idx, int *khz)
{
	if (ch347.pack_size == UNSET)
		ch347.pack_size = ch347_adapter_speed_set(9) ? STANDARD_PACK : LARGER_PACK;

	const int *speeds = ch347.pack_size == STANDARD_PACK ?
		ch347_standard_pack_clock_speeds : ch347_larger_pack_clock_speeds;
	*khz = speeds[speed_idx];
	return ERROR_OK;
}

/**
 * @brief multiplies the input speed by 1000
 *
 * @param khz Speed in kHz
 * @param speed_idx CH347 speed index
 * @return ERROR_OK at success; ERROR_FAIL if khz is zero
 */
static int ch347_speed_get_index(int khz, int *speed_idx)
{
	if (khz == 0) {
		LOG_ERROR("Adaptive clocking not supported");
		return ERROR_FAIL;
	}

	// when checking with speed index 9 we can see if the device supports STANDARD_PACK or LARGER_PACK mode
	if (ch347.pack_size == UNSET)
		ch347.pack_size = ch347_adapter_speed_set(9) ? STANDARD_PACK : LARGER_PACK;

	// depending on pack size there are different fixed clock speeds possible
	const int *speeds = ch347.pack_size == STANDARD_PACK ?
		ch347_standard_pack_clock_speeds : ch347_larger_pack_clock_speeds;
	int length = ch347.pack_size == STANDARD_PACK ?
		ARRAY_SIZE(ch347_standard_pack_clock_speeds) : ARRAY_SIZE(ch347_larger_pack_clock_speeds);
	int idx = -1;
	int lower_bound = 0;

	// find the suitable speed index
	for (int i = 0; i < length; i++) {
		if (khz >= lower_bound && khz <= speeds[i]) {
			idx = i;
			break;
		}
		lower_bound = speeds[i];
	}

	// too high! => use max possible speed
	if (idx == -1) {
		LOG_INFO("Speed %d kHz is higher than highest speed of %d kHz. Using %d khz!",
			khz, speeds[length - 1], speeds[length - 1]);
		idx = length - 1;
	} else if (speeds[idx] != khz) {
		LOG_INFO("Requested speed of %d kHz is not possible. Using the next higher speed of %d kHz!",
			khz, speeds[idx]);
	}

	*speed_idx = idx;
	return ERROR_OK;
}

/**
 * @brief The command handler for setting the device usb vid/pid
 *
 * @return ERROR_OK at success; ERROR_COMMAND_SYNTAX_ERROR otherwise
 */
COMMAND_HANDLER(ch347_handle_vid_pid_command)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], ch347_vids[0]);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], ch347_pids[0]);

	return ERROR_OK;
}

/**
 * @brief The command handler for resetting the target device via trst pin
 *
 * @return Always ERROR_OK
 */
COMMAND_HANDLER(ch347_trst)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ch347_trst_set(false);
	jtag_sleep(atoi(CMD_ARGV[0]) * 1000);
	ch347_trst_set(true);
	return ERROR_OK;
}

/**
 * @brief The command handler for setting the device description that should be found
 *
 * @return ERROR_OK at success; ERROR_COMMAND_SYNTAX_ERROR otherwise
 */
COMMAND_HANDLER(ch347_handle_device_desc_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	free(ch347_device_desc);
	ch347_device_desc = strdup(CMD_ARGV[0]);
	return ERROR_OK;
}

/**
 * @brief The command handler for configuring which GPIO pin is used as activity LED
 *
 * @return ERROR_OK at success; ERROR_COMMAND_SYNTAX_ERROR otherwise
 */
COMMAND_HANDLER(ch347_handle_activity_led_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint8_t gpio;
	if (CMD_ARGV[0][0] == 'n') {
		COMMAND_PARSE_NUMBER(u8, &CMD_ARGV[0][1], gpio);
		ch347_activity_led_active_high = false;
	} else {
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], gpio);
		ch347_activity_led_active_high = true;
	}

	if (gpio >= GPIO_CNT || (BIT(gpio) & USEABLE_GPIOS) == 0)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	ch347_activity_led_gpio_pin = gpio;

	return ERROR_OK;
}

static const struct command_registration ch347_subcommand_handlers[] = {
	{
		.name = "vid_pid",
		.handler = &ch347_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor ID and product ID of the CH347 device",
		.usage = "vid pid",
	},
	{
		.name = "jtag_ntrst_delay",
		.handler = &ch347_trst,
		.mode = COMMAND_ANY,
		.help = "resets via trst pin, parameter is the delay between low and high edge",
		.usage = "[milliseconds]",
	},
	{
		.name = "device_desc",
		.handler = &ch347_handle_device_desc_command,
		.mode = COMMAND_CONFIG,
		.help = "set the USB device description of the CH347 device",
		.usage = "description_string",
	},
	{
		.name = "activity_led",
		.handler = &ch347_handle_activity_led_command,
		.mode = COMMAND_CONFIG,
		.help = "if set this CH347 GPIO pin is the JTAG activity LED; start with n for active low output",
		.usage = "[n]gpio_number",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration ch347_command_handlers[] = {
	{
		.name = "ch347",
		.mode = COMMAND_ANY,
		.help = "perform ch347 management",
		.chain = ch347_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

/**
 * @brief swd init function
 *
 * @return ERROR_OK on success
 */
static int ch347_swd_init_cmd(void)
{
	ch347_cmd_start_next(CH347_CMD_SWD_INIT);
	uint8_t cmd_data[] = {0x40, 0x42, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00 };
	ch347_scratchpad_add_bytes(cmd_data, ARRAY_SIZE(cmd_data));
	/* TODO: CH347_CMD_SWD_INIT reads one data byte.
		But how can we decide if SWD init was successfully executed?
		Return an error code if init was failed */
	ch347_single_read_get_byte(0);
	return ERROR_OK;
}

/**
 * @brief CH347 Initialization function
 *
 * @return ERROR_OK on success
 */
static int ch347_init(void)
{
	int retval = ch347_open_device();

	if (retval != ERROR_OK) {
		LOG_ERROR("CH347 open error");
		return retval;
	}

	LOG_DEBUG_IO("CH347 open success");

	// ch347 jtag init
	ch347.tck_pin = TCK_L;
	ch347.tms_pin = TMS_H;
	ch347.tdi_pin = TDI_L;
	ch347.trst_pin = TRST_H;

	INIT_LIST_HEAD(&ch347.cmd_queue);
	INIT_LIST_HEAD(&ch347.scan_queue);

	ch347.pack_size = UNSET;

	if (!swd_mode)
		tap_set_state(TAP_RESET);
	else
		retval = ch347_swd_init_cmd();

	return retval;
}

/**
 * @brief Initialization for the swd mode
 *
 * @return Always ERROR_OK
 */
static int ch347_swd_init(void)
{
	LOG_INFO("CH347 SWD mode enabled");
	swd_mode = true;
	memset(&ch347_swd_context, 0, sizeof(ch347_swd_context));

	INIT_LIST_HEAD(&ch347_swd_context.send_cmd_head);
	INIT_LIST_HEAD(&ch347_swd_context.free_cmd_head);

	ch347_swd_context.queued_retval = ERROR_OK;
	// 0XE8 + 2byte len + N byte cmds
	ch347_swd_context.send_len = CH347_CMD_HEADER;
	// 0XE8 + 2byte len + N byte ack + data
	ch347_swd_context.need_recv_len = CH347_CMD_HEADER;
	struct ch347_swd_io *pswd_io = ch347_swd_context.ch347_cmd_buf;
	for (int i = 0; i < CH347_MAX_CMD_BUF; i++, pswd_io++) {
		INIT_LIST_HEAD(&pswd_io->list_entry);
		list_add_tail(&pswd_io->list_entry, &ch347_swd_context.free_cmd_head);
	}
	return ERROR_OK;
}

static struct ch347_swd_io *ch347_get_one_swd_io(void)
{
	struct ch347_swd_io *pswd_io;
	if (list_empty(&ch347_swd_context.free_cmd_head))
		return NULL;

	pswd_io = list_first_entry(&ch347_swd_context.free_cmd_head,
				   struct ch347_swd_io, list_entry);
	list_del_init(&pswd_io->list_entry);
	pswd_io->cmd = 0;
	pswd_io->usb_cmd = CH347_CMD_SWD_SEQ_W;
	pswd_io->dst = NULL;
	return pswd_io;
}

static void ch347_swd_queue_flush(void)
{
	int length = ch347_swd_context.send_len;
	ch347_swd_context.send_buf[0] = (uint8_t)CH347_CMD_SWD;
	ch347_swd_context.send_buf[1] = (uint8_t)(ch347_swd_context.send_len - CH347_CMD_HEADER);
	ch347_swd_context.send_buf[2] = (uint8_t)((ch347_swd_context.send_len - CH347_CMD_HEADER) >> 8);
	if (ch347_write_data(ch347_swd_context.send_buf, &length) != ERROR_OK) {
		ch347_swd_context.queued_retval = ERROR_FAIL;
		LOG_DEBUG("CH347WriteData error");
		return;
	}

	ch347_swd_context.recv_len = 0;
	length = ch347_swd_context.recv_len;
	if (ch347_read_data(&ch347_swd_context.recv_buf[ch347_swd_context.recv_len], &length) != ERROR_OK) {
		ch347_swd_context.queued_retval = ERROR_FAIL;
		LOG_DEBUG("CH347ReadData error");
		return;
	}

	ch347_swd_context.recv_len += length;
	if (ch347_swd_context.need_recv_len > ch347_swd_context.recv_len) {
		LOG_ERROR("write/read failed %d %d",
			  ch347_swd_context.recv_len,
			  ch347_swd_context.need_recv_len);
	}
}

static void ch347_write_swd_reg(uint8_t cmd, const uint32_t out)
{
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = CH347_CMD_SWD_REG_W;
	// 8bit + 32bit +1bit
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x29;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x00;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = cmd;
	h_u32_to_le(&ch347_swd_context.send_buf[ch347_swd_context.send_len], out);
	ch347_swd_context.send_len += 4;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = parity_u32(out);
	// 0xA0 +  1 byte(3bit ACK)
	ch347_swd_context.need_recv_len += (1 + 1);
}

static void ch347_write_spec_seq(const uint8_t *out, uint8_t out_len)
{
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] =
		CH347_CMD_SWD_SEQ_W;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out_len;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x00;
	for (uint8_t i = 0; i < DIV_ROUND_UP(out_len, 8); i++)
		ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out ? out[i] : 0x00;
	ch347_swd_context.need_recv_len += 1; /* 0xA1 */
}

static void ch347_read_swd_reg(uint8_t cmd)
{
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = CH347_CMD_SWD_REG_R;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x22;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x00;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = cmd;
	// 0xA2 + 1 byte(3bit ACK) + 4 byte(data) + 1 byte(1bit parity+1bit trn)
	ch347_swd_context.need_recv_len += 1 + 1 + 4 + 1;
}

static int ch347_swd_switch_out(enum swd_special_seq seq, const uint8_t *out, unsigned int out_len)
{
	if ((ch347_swd_context.send_len + (1 + 2 + DIV_ROUND_UP(out_len, 8))) > CH347_MAX_SEND_BUF)
		return ERROR_FAIL;
	if ((ch347_swd_context.need_recv_len + 2) > CH347_MAX_RECV_BUF)
		return ERROR_FAIL;

	struct ch347_swd_io *pswd_io = ch347_get_one_swd_io();
	if (pswd_io) {
		ch347_write_spec_seq(out, out_len);
		list_add_tail(&pswd_io->list_entry, &ch347_swd_context.send_cmd_head);
		return ERROR_OK;
	} else {
		return ERROR_FAIL;
	}
}

// check read/write REG can fill in remaining buff
static bool ch347_chk_buf_size(uint8_t cmd, uint32_t ap_delay_clk)
{
	bool flush = false;
	int send_len = ch347_swd_context.send_len;
	int recv_len = ch347_swd_context.need_recv_len;
	int len;
	do {
		if (cmd & SWD_CMD_RNW) {
			len = 1 + 1 + 1 + 1; // 0xA2 + len + rev + cmd
			if (send_len + len > CH347_MAX_SEND_BUF)
				break;
			send_len += len;
			len = 1 + 1 + 4 + 1;
			/* 0xA2 + 1byte(3bit ack) +  4byte(data) +
			   1byte(1bit parity+1bit trn) */
			if (recv_len + len > CH347_MAX_RECV_BUF)
				break;
			recv_len += len;
		} else {                         // write reg
			len = 1 + 1 + 1 + 1 + 4 + 1;
			// 0xA0 + len + rev  + cmd +data + parity
			if (send_len + len > CH347_MAX_SEND_BUF)
				break;
			send_len += len;
			len = 1 + 1; // 0xA0 + 1byte(3bit ack)
			if (recv_len + len > CH347_MAX_RECV_BUF)
				break;
			recv_len += len;
		}
		if (cmd & SWD_CMD_APNDP) {
			len = 1 + 1 + 1 + DIV_ROUND_UP(ap_delay_clk, 8);
			// 0xA1 + Len  + rev  + n byte(delay)
			if (send_len + len > CH347_MAX_SEND_BUF)
				break;
			len = 1; // 0xA1
			if ((recv_len + len) > CH347_MAX_RECV_BUF)
				break;
		}
		// swd packet requests
		flush = true;
	} while (false);

	return flush;
}

// foreward declarations
static int ch347_swd_run_queue(void);

static void ch347_swd_send_idle(uint32_t ap_delay_clk)
{
	struct ch347_swd_io *pswd_io = ch347_get_one_swd_io();
	if (!pswd_io) {
		ch347_swd_run_queue();
		pswd_io = ch347_get_one_swd_io();
		if (!pswd_io) {
			LOG_DEBUG("ch347_swd_queue_cmd error");
			ch347_swd_context.queued_retval = ERROR_FAIL;
			return;
		}
	}
	ch347_write_spec_seq(NULL, ap_delay_clk);

	list_add_tail(&pswd_io->list_entry, &ch347_swd_context.send_cmd_head);
}

static int ch347_swd_run_queue(void)
{
	LOG_DEBUG_IO("Executing %u queued transactions", ch347_swd_context.sent_cmd_count);
	if (ch347_swd_context.queued_retval != ERROR_OK) {
		LOG_DEBUG_IO("Skipping due to previous errors: %d", ch347_swd_context.queued_retval);
		goto skip;
	}

	/* A transaction must be followed by another transaction or at least 8
	   idle cycles to ensure that data is clocked through the AP. */
	if ((ch347_swd_context.send_len + (1 + 2 + 1)) > CH347_MAX_SEND_BUF)
		goto skip_idle;

	if ((ch347_swd_context.need_recv_len + 1) > CH347_MAX_RECV_BUF)
		goto skip_idle;

	ch347_swd_send_idle((uint32_t)8);

skip_idle:

	ch347_swd_queue_flush();

	if (ch347_swd_context.queued_retval != ERROR_OK) {
		LOG_ERROR("CH347 usb write/read failed - queued_retval");
		goto skip;
	}
	uint8_t *recv_buf = ch347_swd_context.recv_buf;
	int recv_len = 0;
	if (recv_buf[recv_len++] != CH347_CMD_SWD) { // 0XE8
		ch347_swd_context.queued_retval = ERROR_FAIL;
		LOG_ERROR("CH347 usb write/read failed - not CH347_CMD_SWD");
		goto skip;
	}

	int cmds_len = le_to_h_u16(&recv_buf[recv_len]);
	recv_len += 2; // cmds_len
	if ((cmds_len + CH347_CMD_HEADER) > ch347_swd_context.recv_len) {
		ch347_swd_context.queued_retval = ERROR_FAIL;
		LOG_ERROR("CH347 usb write/read failed - too long");
		goto skip;
	}

	struct list_head *tmp;
	struct list_head *pos;
	struct ch347_swd_io *pswd_io;

	list_for_each_safe(pos, tmp, &ch347_swd_context.send_cmd_head) {
		pswd_io = list_entry(pos, struct ch347_swd_io, list_entry);
		if (pswd_io->usb_cmd == CH347_CMD_SWD_SEQ_W) {
			if (recv_buf[recv_len++] != CH347_CMD_SWD_SEQ_W) {
				ch347_swd_context.queued_retval = ERROR_FAIL;
				LOG_ERROR("CH347 usb write/read failed - not CH347_CMD_SWD_SEQ_W");
				goto skip;
			}
		} else { // read/write Reg
			uint32_t ack;
			bool check_ack;
			// read  Reg
			if (recv_buf[recv_len] == CH347_CMD_SWD_REG_R) {
				recv_len++;
				ack = buf_get_u32(&recv_buf[recv_len++], 0, 3);
				/* Devices do not reply to DP_TARGETSEL write
				   cmd, ignore received ack */
				check_ack = swd_cmd_returns_ack(pswd_io->cmd);
				if (ack != SWD_ACK_OK && check_ack) {
					ch347_swd_context.queued_retval = swd_ack_to_error_code(ack);
					LOG_ERROR("ack != SWD_ACK_OK");
					goto skip;
				}
				if (pswd_io->cmd & SWD_CMD_RNW) {
					uint32_t data = buf_get_u32(&recv_buf[recv_len], 0, 32);
					uint32_t parity = buf_get_u32(&recv_buf[recv_len], 32, 1);
					if (parity != (uint32_t)parity_u32(data)) {
						LOG_ERROR("SWD Read data parity mismatch");
						ch347_swd_context.queued_retval = ERROR_FAIL;
						goto skip;
					}

					LOG_DEBUG_IO("%s%s %s %s reg %X = %08X - " PRIx32,
							 check_ack ? "" : "ack ignored ",
							 ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" :
								ack == SWD_ACK_FAULT  ? "FAULT" : "JUNK",
							 pswd_io->cmd & SWD_CMD_APNDP ? "AP" : "DP",
							 pswd_io->cmd & SWD_CMD_RNW ? "read" : "write",
							 (pswd_io->cmd & SWD_CMD_A32) >> 1,
							 data);

					if (pswd_io->dst)
						*pswd_io->dst = data;
				} else {
					ch347_swd_context.queued_retval = ERROR_FAIL;
					LOG_ERROR("CH347 usb write/read failed - not SWD_CMD_RNW");
					goto skip;
				}
				recv_len += 5;
			} else if (recv_buf[recv_len] == CH347_CMD_SWD_REG_W) {
				recv_len++;
				ack = buf_get_u32(&recv_buf[recv_len++], 0, 3);
				/* Devices do not reply to DP_TARGETSEL write
				   cmd, ignore received ack */
				check_ack = swd_cmd_returns_ack(pswd_io->cmd);
				if (ack != SWD_ACK_OK && check_ack) {
					ch347_swd_context.queued_retval = swd_ack_to_error_code(ack);
					LOG_ERROR("SWD Read data parity mismatch");
					goto skip;
				}
				LOG_DEBUG_IO("%s%s %s %s reg %X = %08X - " PRIx32,
						 check_ack ? "" : "ack ignored ",
						 ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" :
							ack == SWD_ACK_FAULT  ? "FAULT" : "JUNK",
						 pswd_io->cmd & SWD_CMD_APNDP ? "AP" : "DP",
						 pswd_io->cmd & SWD_CMD_RNW ? "read" : "write",
						 (pswd_io->cmd & SWD_CMD_A32) >> 1,
						 pswd_io->value);
			} else {
				ch347_swd_context.queued_retval = ERROR_FAIL;
				LOG_ERROR("CH347 usb write/read failed recv_len = %d", recv_len);
				goto skip;
			}
		}
		list_del_init(&pswd_io->list_entry);
		list_add_tail(&pswd_io->list_entry,
				  &ch347_swd_context.free_cmd_head);
	}

skip:
	if (!list_empty(&ch347_swd_context.send_cmd_head)) {
		list_for_each_safe(pos, tmp, &ch347_swd_context.send_cmd_head) {
			pswd_io = list_entry(pos, struct ch347_swd_io, list_entry);
			list_del_init(&pswd_io->list_entry);
			list_add_tail(&pswd_io->list_entry,
					  &ch347_swd_context.free_cmd_head);
		}
	}
	// 0xE8 + 2byte len
	ch347_swd_context.send_len = CH347_CMD_HEADER;
	// 0xE8 + 2byte len
	ch347_swd_context.need_recv_len = CH347_CMD_HEADER;
	ch347_swd_context.recv_len = 0;
	ch347_swd_context.sent_cmd_count = 0;
	int retval = ch347_swd_context.queued_retval;
	ch347_swd_context.queued_retval = ERROR_OK;

	return retval;
}

static void ch347_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data, uint32_t ap_delay_clk)
{
	if (ap_delay_clk > 255)
		LOG_DEBUG("ap_delay_clk = %d", ap_delay_clk);

	if (ch347_swd_context.sent_cmd_count >= CH347_MAX_SEND_CMD)
		ch347_swd_run_queue();

	if (!ch347_chk_buf_size(cmd, ap_delay_clk))
		ch347_swd_run_queue();

	struct ch347_swd_io *pswd_io = ch347_get_one_swd_io();
	if (!pswd_io) {
		ch347_swd_run_queue();
		pswd_io = ch347_get_one_swd_io();
		if (!pswd_io) {
			LOG_DEBUG("Error");
			ch347_swd_context.queued_retval = ERROR_FAIL;
			return;
		}
	}

	pswd_io->cmd = cmd | SWD_CMD_START | SWD_CMD_PARK;

	if (pswd_io->cmd & SWD_CMD_RNW) {
		pswd_io->usb_cmd = CH347_CMD_SWD_REG_R;
		pswd_io->dst = dst;
		ch347_read_swd_reg(pswd_io->cmd);
	} else {
		pswd_io->usb_cmd = CH347_CMD_SWD_REG_W;
		pswd_io->value = data;
		ch347_write_swd_reg(pswd_io->cmd, data);
	}

	ch347_swd_context.sent_cmd_count++;
	list_add_tail(&pswd_io->list_entry, &ch347_swd_context.send_cmd_head);
	// Insert idle cycles after AP accesses to avoid WAIT
	if (cmd & SWD_CMD_APNDP) {
		if (ap_delay_clk == 0)
			LOG_DEBUG("ap_delay_clk == 0");
		ch347_swd_send_idle(ap_delay_clk);
	}
}

static int ch347_swd_switch_seq(enum swd_special_seq seq)
{
	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		return ch347_swd_switch_out(seq, swd_seq_line_reset, swd_seq_line_reset_len);
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		return ch347_swd_switch_out(seq, swd_seq_jtag_to_swd, swd_seq_jtag_to_swd_len);
	case JTAG_TO_DORMANT:
		LOG_DEBUG("JTAG-to-DORMANT");
		return ch347_swd_switch_out(seq, swd_seq_jtag_to_dormant, swd_seq_jtag_to_dormant_len);
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		return ch347_swd_switch_out(seq, swd_seq_swd_to_jtag, swd_seq_swd_to_jtag_len);
	case SWD_TO_DORMANT:
		LOG_DEBUG("SWD-to-DORMANT");
		return ch347_swd_switch_out(seq, swd_seq_swd_to_dormant, swd_seq_swd_to_dormant_len);
	case DORMANT_TO_SWD:
		LOG_DEBUG("DORMANT-to-SWD");
		return ch347_swd_switch_out(seq, swd_seq_dormant_to_swd, swd_seq_dormant_to_swd_len);
	case DORMANT_TO_JTAG:
		LOG_DEBUG("DORMANT-to-JTAG");
		return ch347_swd_switch_out(seq, swd_seq_dormant_to_jtag, swd_seq_dormant_to_jtag_len);
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}
}

static void ch347_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RNW);
	ch347_swd_queue_cmd(cmd, value, 0, ap_delay_clk);
}

static void ch347_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RNW));
	ch347_swd_queue_cmd(cmd, NULL, value, ap_delay_clk);
}

static const struct swd_driver ch347_swd = {
	.init = ch347_swd_init,
	.switch_seq = ch347_swd_switch_seq,
	.read_reg = ch347_swd_read_reg,
	.write_reg = ch347_swd_write_reg,
	.run = ch347_swd_run_queue,
};

static const char *const ch347_transports[] = {"jtag", "swd", NULL};

static struct jtag_interface ch347_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = ch347_execute_queue,
};

struct adapter_driver ch347_adapter_driver = {
	.name = "ch347",
	.transports = ch347_transports,
	.commands = ch347_command_handlers,

	.init = ch347_init,
	.quit = ch347_quit,
	.reset = ch347_reset_assert,
	.speed = ch347_speed_set,
	.khz = ch347_speed_get_index,
	.speed_div = ch347_speed_get,

	.jtag_ops = &ch347_interface,
	.swd_ops = &ch347_swd,
};
