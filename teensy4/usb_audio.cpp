/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include "usb_dev.h"
#include "usb_audio.h"
#include "debug/printf.h"

#ifdef AUDIO_INTERFACE

bool AudioInputUSB::update_responsibility;
audio_block_t * AudioInputUSB::incoming[AUDIO_CHANNELS];
audio_block_t * AudioInputUSB::ready[AUDIO_CHANNELS];


// The current amount that the incoming (pending) buffer is full. Between 0 and AUDIO_BLOCK_SAMPLES.
uint16_t AudioInputUSB::incoming_count; 

// A flag that there's data available for the next AudioStream consumer.
uint8_t AudioInputUSB::receive_flag;

struct usb_audio_features_struct AudioInputUSB::features = {0,0,FEATURE_MAX_VOLUME/2};

extern volatile uint8_t usb_high_speed;
static void rx_event(transfer_t *t);
static void tx_event(transfer_t *t);

/*static*/ transfer_t rx_transfer __attribute__ ((used, aligned(32)));
/*static*/ transfer_t sync_transfer __attribute__ ((used, aligned(32)));
/*static*/ transfer_t tx_transfer __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t rx_buffer[AUDIO_RX_SIZE] __attribute__ ((aligned(32)));
DMAMEM uint32_t usb_audio_sync_feedback __attribute__ ((aligned(32)));

uint8_t usb_audio_receive_setting=0;
uint8_t usb_audio_transmit_setting=0;
uint8_t usb_audio_sync_nbytes;
uint8_t usb_audio_sync_rshift;

#define SINK_DATARATE_BUF_SIZE 2
volatile int16_t sink_datarate_buf[SINK_DATARATE_BUF_SIZE] = {0};
volatile uint32_t sink_datarate_idx;
uint32_t report_sink_offset(int16_t new_rate) {
	sink_datarate_buf[sink_datarate_idx++ % SINK_DATARATE_BUF_SIZE] = new_rate;
}
float get_sink_pressure() {
	char c[100];
	if (sink_datarate_idx < SINK_DATARATE_BUF_SIZE) {
		return 0.0f;
	}

	float avg = 0.0f;
	for (uint16_t i; i < SINK_DATARATE_BUF_SIZE; i++) {
		avg += (float)sink_datarate_buf[i] / (float)SINK_DATARATE_BUF_SIZE;
	}

	return avg;
}

// In the USB documentation:
// Fs: The *actual* sample rate currently witnessed, as measured relative to the USB (micro)frames SOF.
//     so, for Full-Speed that would be every 1ms, and for High-Speed every 125us (8x faster).
// Ff: The *desired* data rate to achieve a target sample rate.
//
// In this case, the feedback accumulator is updated based on how quickly samples are consumed down the
// audio chain (for example, by an I2S output), which is why it's modified in the "update" function,
// and read in the sync_even function.
volatile uint32_t feedback_accumulator;

// A flag to mark the initial buffer has been filled enough once USB data has started
// to consider any underflows *actual* underflows.
volatile uint8_t buffer_floodgate_open = 0;

volatile uint32_t usb_audio_underrun_count = 0, usb_audio_overrun_count = 0;
volatile uint32_t sync_counter = 0, callback_counter = 0;

static void rx_event(transfer_t *t)
{
	if (t) {
		int len = AUDIO_PACKET_SIZE(AUDIO_RX_SIZE) - ((rx_transfer.status >> 16) & 0x7FFF);
		// printf("rx %u\n", len);
		usb_audio_receive_callback(len);
	}
	usb_prepare_transfer(&rx_transfer, rx_buffer, AUDIO_RX_SIZE, 0);
	arm_dcache_delete(&rx_buffer, AUDIO_RX_SIZE);
	usb_receive(AUDIO_RX_ENDPOINT, &rx_transfer);
}

static void sync_event(transfer_t *t)
{
	sync_counter++;
	// USB 2.0 Specification, 5.12.4.2 Feedback, pages 73-75
	usb_audio_sync_feedback = feedback_accumulator >> usb_audio_sync_rshift;
	usb_prepare_transfer(&sync_transfer, &usb_audio_sync_feedback, usb_audio_sync_nbytes, 0);
	arm_dcache_flush(&usb_audio_sync_feedback, usb_audio_sync_nbytes);
	usb_transmit(AUDIO_SYNC_ENDPOINT, &sync_transfer);
}

void usb_audio_configure(void)
{
	printf("usb_audio_configure\n");
	buffer_floodgate_open = 0;
	usb_audio_underrun_count = 0;
	usb_audio_overrun_count = 0;

	// The feedback accumulator keeps track of how many samples have been seen
	// based on the USB Host's clock, which for USB FS triggers every 1ms, and for
	// USB HS triggers 8 times per 1ms (every 125us).
	//
	// We're multiplying by 2^24 here as a convenience for formatting the response
	// to the host, which expects:
	//   if High-Speed (480Mbps): An unsigned 10.10 fixed point binary, aligned as a 3-byte 10.14.
	//   if Full-Speed ( 12Mbps): An unsigned 10.14 fixed point binary, aligned as a 4-byte 16.16.
	// and shifting down a dynamic number of bytes to match that format.
	feedback_accumulator = (AUDIO_SAMPLE_RATE_EXACT / 1000.0f) * 0x1000000; // samples/millisecond * 2^24
	// USB 2.0 High-Speed uses a 4-byte feedback format,
	// where Full Speed uses a 3-byte format.
	if (usb_high_speed) {
		usb_audio_sync_nbytes = 4;
		usb_audio_sync_rshift = 8;
	} else {
		usb_audio_sync_nbytes = 3;
		usb_audio_sync_rshift = 10;
	}
	memset(&rx_transfer, 0, sizeof(rx_transfer));
	usb_config_rx_iso(AUDIO_RX_ENDPOINT, AUDIO_RX_SIZE, 1, rx_event);
	rx_event(NULL);
	memset(&sync_transfer, 0, sizeof(sync_transfer));
	usb_config_tx_iso(AUDIO_SYNC_ENDPOINT, usb_audio_sync_nbytes, 1, sync_event);
	sync_event(NULL);
	memset(&tx_transfer, 0, sizeof(tx_transfer));
	usb_config_tx_iso(AUDIO_TX_ENDPOINT, AUDIO_TX_SIZE, 1, tx_event);
	tx_event(NULL);
}

void AudioInputUSB::begin(void)
{
	incoming_count = 0;
	for (int i = 0; i < AUDIO_CHANNELS; i++) {
		incoming[i] = NULL;
		ready[i] = NULL;
	}
	receive_flag = 0;
	update_responsibility = false;
}
static void copy_to_buffers(const uint32_t *src, audio_block_t *chans[AUDIO_CHANNELS], unsigned int count, unsigned int len) 
{
	uint32_t *target = (uint32_t*) src + (len * AUDIO_CHANNELS/2);

	uint32_t i = 0;
	while ((src < target)) {
		for (unsigned int j = 0; j < AUDIO_CHANNELS/2; j++) {
			uint32_t n = *src++;
			chans[j*2]->data[count+i] = n & 0xFFFF;
			// NOTE: inverting this sample here because for some reason
			// one of the channels has a phase issue...
			chans[j*2+1]->data[count+i] = UINT16_MAX - (n >> 16);
			// chans[j*2+1]->data[count+i] = n >> 16;
		}
		i++;
	}
}

// Called from the USB interrupt when an isochronous packet arrives
// we must completely remove it from the receive buffer before returning
//
#if 1
void usb_audio_receive_callback(unsigned int len)
{
	unsigned int count, avail;
	audio_block_t *chans[AUDIO_CHANNELS];
	const uint16_t *data;
	const uint32_t *data_orig;

	AudioInputUSB::receive_flag = 1;
	// Let `len` now represent the number of "frames" of audio
	// One frame includes exactly one sample per channel.
	//
	// For example, if we have 8 channels of 16-bit audio,
	// one frame would be 16 bytes long, 8 * 2 bytes.
	len /= AUDIO_CHANNELS * AUDIO_SAMPLE_BYTES;
	callback_counter++;

	// A moving pointer to the USB receive buffer as we eat it up.
	data_orig = (const uint32_t *)rx_buffer;

	count = AudioInputUSB::incoming_count;

	// Either use the existing incoming buffers or allocate new ones as needed.
	for (uint32_t i = 0; i < AUDIO_CHANNELS; i++) {
		chans[i] = AudioInputUSB::incoming[i];
	}
	for (uint32_t i = 0; i < AUDIO_CHANNELS; i++) {
		if (AudioInputUSB::incoming[i] == NULL) {
			chans[i] = AudioStream::allocate();
			if(chans[i] == NULL) goto cleanup;
			AudioInputUSB::incoming[i] = chans[i];
		}
	}
	while (len > 0) {
		avail = AUDIO_BLOCK_SAMPLES - count;
		if (len < avail) {
			// We can fit the entire incoming buffer in one AudioStream block,
			// so we simply copy the whole thing in.
			CORE_PIN6_PORTSET = CORE_PIN6_BITMASK;
			copy_to_buffers(data_orig, chans, count, len);
			CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;
			AudioInputUSB::incoming_count = count + len;
			goto cleanup;
		} else if (avail > 0) {
			// Otherwise, we will be finishing up filling a block, and can
			// flush it to the next consumer in the chain.
			//
			// We'll need to finish consuming the data after that, though,
			// which is why this looks complicated.
			CORE_PIN6_PORTSET = CORE_PIN6_BITMASK;
			copy_to_buffers(data_orig, chans, count, avail);
			CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;
			data_orig += avail * AUDIO_CHANNELS / AUDIO_SAMPLE_BYTES;
			len -= avail;
			for (int i = 0; i < AUDIO_CHANNELS; i++) {
				if (AudioInputUSB::ready[i]) {
					// If the previous ready buffers have not been consumed,
					// all we can do is fill up our current incoming buffer
					// and wait to be able to swap them.
					AudioInputUSB::incoming_count = count + avail;
					if (len > 0) {
						// If there were remaining bytes of audio, they will
						// be dropped because there is nowhere to put them.
						usb_audio_overrun_count++;
						char c[20];
						float accumulator = (float)(feedback_accumulator >> 8) / (float)(1<<16);
						snprintf(c, 20, "%.6f", accumulator);
						printf("^ OVERRUN, accumulator: %skHz, len: %d, i: %d\n", c, len, i);
						//serial_phex(len);
					}
					goto cleanup;
				}
			}
			send:
			for (int i = 0; i < AUDIO_CHANNELS; i++) {
				AudioInputUSB::ready[i] = chans[i];
			}
			buffer_floodgate_open = 1;
			for (int i = 0; i < AUDIO_CHANNELS; i++) {
				chans[i] = AudioStream::allocate();
				if (chans[i] == NULL) {
					for (int k = 0; k < i; k++) {
						AudioStream::release(chans[k]);
					}
					for (int j = 0; j < AUDIO_CHANNELS; j++) {
						AudioInputUSB::incoming[j] = NULL;
					}
					AudioInputUSB::incoming_count = 0;
					goto cleanup;
				}
			}
			for (int i = 0; i < AUDIO_CHANNELS; i++) {
				AudioInputUSB::incoming[i] = chans[i];
			}
			count = 0;
		} else {
			for (int i = 0; i < AUDIO_CHANNELS; i++) {
				if (AudioInputUSB::ready[i]) {
					goto cleanup;
				}
			}
			goto send; // recover from buffer overrun
		}
	}
	AudioInputUSB::incoming_count = count;
	cleanup:
	CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;
}
#endif

void AudioInputUSB::update(void)
{
	CORE_PIN5_PORTSET = CORE_PIN5_BITMASK;
	// printf("AudioInputUSB::update\n");
	audio_block_t *chans[AUDIO_CHANNELS];

	__disable_irq();

	for (int i = 0; i < AUDIO_CHANNELS; i++) {
		chans[i] = ready[i];
		ready[i] = NULL;
	}
	uint16_t c = incoming_count;
	uint8_t f = receive_flag;
	receive_flag = 0;
	__enable_irq();
	if (f && usb_audio_receive_setting) {
		// Did we receive more or less than half 

		// The amount "off from target" that we are.
		// diff < 0   Receiving more samples than our downstream is consuming, risk of overrun.
		//            We must request a lower Ff (desired data rate).
		// diff > 0   Receiving fewer samples than our downstream is consuming, risk of underrun.
		//            Must request a higher Ff (desired data rate).
		int diff = AUDIO_BLOCK_SAMPLES/2 - (int)c;

		report_sink_offset(diff);
		float pressure = get_sink_pressure();

		// snprintf(c, 99, "avg: %.6f, [%d, %d, %d, %d]", pressure,
		// 	sink_datarate_buf[0], sink_datarate_buf[1], sink_datarate_buf[2], sink_datarate_buf[3]);
		// printf("%s\n", c);

		float pressure_multiplier = 1.0f + (pressure / (AUDIO_BLOCK_SAMPLES / 2) / 10000);
		// if (pressure_multiplier > 1.2f || pressure_multiplier < 0.8f) {
		// 	char c[100];
		// 	snprintf(c, 99, "pressure: %.5f, mult: %.5f",
		// 		pressure, pressure_multiplier);
		// 	printf("%s\n", c);
		// }
		feedback_accumulator = (AUDIO_SAMPLE_RATE_EXACT * pressure_multiplier / 1000.0f) * 0x1000000;
		feedback_accumulator += diff;
	} else if (!usb_audio_receive_setting) {
		feedback_accumulator = (AUDIO_SAMPLE_RATE_EXACT / 1000.0f) * 0x1000000;
	}
	//serial_phex(c);
	//serial_print(".");
	if (usb_audio_receive_setting && buffer_floodgate_open) {
	for (int i = 0; i < AUDIO_CHANNELS; i++) {
		if (!chans[i]) {
			usb_audio_underrun_count++;
				char c[20];
				float accumulator = (float)(feedback_accumulator >> 8) / (float)(1<<16);
				snprintf(c, 20, "%.6f", accumulator);
				printf("v UNDERRUN, accumulator: %skHz, i: %d, receive? %d\n", c, i, usb_audio_receive_setting); // buffer underrun - PC sending too slow
			break;
		}
	}
	}
	for (int i = 0; i < AUDIO_CHANNELS; i++) {
		if (chans[i]) {
			transmit(chans[i], i);
			release(chans[i]);
		}
	}
	// if (!usb_audio_receive_setting) {
	// 	buffer_floodgate_open = 0;
	// 	}
	CORE_PIN5_PORTCLEAR = CORE_PIN5_BITMASK;
}



/*********** AudioOutputUSB *************/
#if 1
bool AudioOutputUSB::update_responsibility;
audio_block_t * AudioOutputUSB::outgoing[AUDIO_CHANNELS]; // being transmitted by USB
audio_block_t * AudioOutputUSB::ready[AUDIO_CHANNELS]; // next in line to be transmitted
uint16_t AudioOutputUSB::offset_1st;
int AudioOutputUSB::normal_target; 
int AudioOutputUSB::accumulator;   
int AudioOutputUSB::subtract;  
/*DMAMEM*/ uint16_t usb_audio_transmit_buffer[AUDIO_TX_SIZE/2] __attribute__ ((used, aligned(32)));


static void tx_event(transfer_t *t)
{
	int len = usb_audio_transmit_callback();
	usb_audio_sync_feedback = feedback_accumulator >> usb_audio_sync_rshift;
	usb_prepare_transfer(&tx_transfer, usb_audio_transmit_buffer, len, 0);
	arm_dcache_flush_delete(usb_audio_transmit_buffer, len);
	usb_transmit(AUDIO_TX_ENDPOINT, &tx_transfer);
}


void AudioOutputUSB::begin(void)
{
	update_responsibility = false;
	for (int i =0;i<AUDIO_CHANNELS;i++)
	{
		outgoing[i] = NULL;
		ready[i] = NULL;
	}
	
	// preset sample rate fine-tuning: assumes rate is an integer number of samples per second
	int txFreq = 1000 * (1 << (4 - AUDIO_INTERVAL(AUDIO_TX_SIZE)));
	normal_target = (int) ((AUDIO_FREQUENCY) / txFreq); 		// at least this many samples per millisecond 
	accumulator = txFreq / 2; 									// start half-full
	subtract = (int) (AUDIO_FREQUENCY) - normal_target*txFreq;	// accumulate error this fast
}

static void copy_from_buffers(uint32_t *dst, int16_t *left, int16_t *right, unsigned int len)
{
	// TODO: optimize...
	while (len > 0) {
		*dst++ = (*right++ << 16) | (*left++ & 0xFFFF);
		len--;
	}
}

/*
 * On update(), we just receive the audio blocks and keep a set of pointers
 * to them. The USB transmit callback will then copy them to the transmit buffer
 * and release them at some point in the future.
 */
void AudioOutputUSB::update(void)
{
	audio_block_t* chans[AUDIO_CHANNELS];
	int i;
	

	// get the audio data
	for (i=0;i<AUDIO_CHANNELS;i++)
		chans[i] = receiveReadOnly(i);
	
	if (usb_audio_transmit_setting == 0) // not transmitting: dump all audio data
	{
		__disable_irq(); // avoid issues if USB interrupt occurs during this process
		
		for (i=0;i<AUDIO_CHANNELS;i++)
		{
			if (NULL != chans[i]) 
				release(chans[i]);
			if (NULL != outgoing[i]) 
			{
				release(outgoing[i]);
				outgoing[i] = NULL;
			}
				
			if (NULL != ready[i]) 
			{
				release(ready[i]);
				ready[i] = NULL;
			}
		}
		__enable_irq();
		offset_1st = 0;
	}
	else
	{
		// ensure every channel has a real audio block, even if it's silent
		for (i=0;i<AUDIO_CHANNELS;i++)
		{
			if (NULL == chans[i]) // sent NULL: make implied silence into real data
			{
				chans[i] = allocate();
				if (NULL != chans[i])
					memset(chans[i]->data, 0, sizeof(chans[i]->data));
				else
					break; // no block available, exit early
			}
		}
		
		if (i >= AUDIO_CHANNELS) // no invalid audio, queue all for transmission
		{
			__disable_irq();
			
			if (NULL == outgoing[0]) // just (re-)starting
			{
				for (i=0;i<AUDIO_CHANNELS;i++)
					outgoing[i] = chans[i];
				offset_1st = 0;
			} 
			else if (NULL == ready[0]) 
			{
				for (i=0;i<AUDIO_CHANNELS;i++)
					ready[i] = chans[i];
			} 
			else 
			{
				// buffer overrun - PC is consuming too slowly
				for (i=0;i<AUDIO_CHANNELS;i++)
				{
					audio_block_t* discard = outgoing[i];
					outgoing[i] = ready[i];
					ready[i] = chans[i];
					release(discard);
				}
				offset_1st = 0; // TODO: discard part of this data?
			}
			__enable_irq();
		}
		else // some invalid audio, can't queue any - discard it all
		{
			for (i=0;i<AUDIO_CHANNELS;i++)
				if (NULL != chans[i])
					release(chans[i]);
			
		}
	}
}


static void interleave_from_blocks(int16_t* transmit_buffer,	//!< next free sample in USB transmit buffer
								audio_block_t** outgoing, //!< array of pointers to source audio blocks
								int chans, //!< number of entries in the array
								int offset, //!< sample# of next "fresh" sample
								int num) //!< number of samples to copy
{
	for (int j=0;j<num;j++)
	{
		for (int i=0;i<chans;i++)
			*transmit_buffer++ = outgoing[i]->data[offset];
		offset++;
	}
}
								
// Called from the USB interrupt when ready to transmit another
// isochronous packet.  If we place data into the transmit buffer,
// the return is the number of bytes.  Otherwise, return 0 means
// no data to transmit
unsigned int usb_audio_transmit_callback(void)
{
	uint32_t avail, num, target = AudioOutputUSB::normal_target, offset, len=0;

	// adjust target number of samples we want to transmit, if needed
	AudioOutputUSB::accumulator -= AudioOutputUSB::subtract;
	if (AudioOutputUSB::accumulator <= 0) // underflowed
	{
		target++; // need to transmit an extra sample this time
		AudioOutputUSB::accumulator += 1000; // bump accumulator back above threshold
	}
	
	while (len < target) // may take two iterations if not enough in outgoing[]
	{
		num = target - len; // number of samples left to transmit
		__disable_irq();
		if (NULL == AudioOutputUSB::outgoing[0]) 
		{
			// buffer underrun - PC is consuming too quickly
			memset(usb_audio_transmit_buffer + len, 0, num * AUDIO_CHANNELS * sizeof AudioOutputUSB::outgoing[0]->data[0]);
			//serial_print("%");
			__enable_irq();
			break;
		}
		offset = AudioOutputUSB::offset_1st;

		avail = AUDIO_BLOCK_SAMPLES - offset;
		if (num > avail) num = avail;

		//copy_from_buffers((uint32_t *)usb_audio_transmit_buffer + len,
		//	left->data + offset, right->data + offset, num);
		
		// have to cast type of transmit buffer, because although samples are actually
		// signed integers, some other modules say they're unsigned...
		interleave_from_blocks((int16_t*) usb_audio_transmit_buffer + len * AUDIO_CHANNELS,
								AudioOutputUSB::outgoing, AUDIO_CHANNELS, offset,
								num);
		__enable_irq();
		
		len += num;
		offset += num;
		if (offset >= AUDIO_BLOCK_SAMPLES) 
		{
			for (int i=0;i<AUDIO_CHANNELS;i++)
			{
				AudioStream::release(AudioOutputUSB::outgoing[i]);
				AudioOutputUSB::outgoing[i] = AudioOutputUSB::ready[i];
				AudioOutputUSB::ready[i] = NULL;
			}
			AudioOutputUSB::offset_1st = 0;
		} 
		else 
		{
			AudioOutputUSB::offset_1st = offset;
		}
	}
	return len * AUDIO_CHANNELS * sizeof AudioOutputUSB::outgoing[0]->data[0];
}
#endif




struct setup_struct {
  union {
    struct {
	uint8_t bmRequestType;
	uint8_t bRequest;
	union {
		struct {
			uint8_t bChannel;  // 0=main, 1=left, 2=right
			uint8_t bCS;       // Control Selector
		};
		uint16_t wValue;
	};
	union {
		struct {
			uint8_t bIfEp;     // type of entity
			uint8_t bEntityId; // UnitID, TerminalID, etc.
		};
		uint16_t wIndex;
	};
	uint16_t wLength;
    };
  };
};

int usb_audio_get_feature(void *stp, uint8_t *data, uint32_t *datalen)
{
	struct setup_struct setup = *((struct setup_struct *)stp);
	if (setup.bmRequestType==0xA1) { // should check bRequest, bChannel, and UnitID
			if (setup.bCS==0x01) { // mute
				data[0] = AudioInputUSB::features.mute;  // 1=mute, 0=unmute
				*datalen = 1;
				return 1;
			}
			else if (setup.bCS==0x02) { // volume
				if (setup.bRequest==0x81) { // GET_CURR
					data[0] = AudioInputUSB::features.volume & 0xFF;
					data[1] = (AudioInputUSB::features.volume>>8) & 0xFF;
				}
				else if (setup.bRequest==0x82) { // GET_MIN
					//serial_print("vol get_min\n");
					data[0] = 0;     // min level is 0
					data[1] = 0;
				}
				else if (setup.bRequest==0x83) { // GET_MAX
					data[0] = FEATURE_MAX_VOLUME;  // max level, for range of 0 to MAX
					data[1] = 0;
				}
				else if (setup.bRequest==0x84) { // GET_RES
					data[0] = 1; // increment vol by by 1
					data[1] = 0;
				}
				else { // pass over SET_MEM, etc.
					return 0;
				}
				*datalen = 2;
				return 1;
			}
	}
	return 0;
}

int usb_audio_set_feature(void *stp, uint8_t *buf) 
{
	struct setup_struct setup = *((struct setup_struct *)stp);
	if (setup.bmRequestType==0x21) { // should check bRequest, bChannel and UnitID
			if (setup.bCS==0x01) { // mute
				if (setup.bRequest==0x01) { // SET_CUR
					AudioInputUSB::features.mute = buf[0]; // 1=mute,0=unmute
					AudioInputUSB::features.change = 1;
					return 1;
				}
			}
			else if (setup.bCS==0x02) { // volume
				if (setup.bRequest==0x01) { // SET_CUR
					AudioInputUSB::features.volume = buf[0];
					AudioInputUSB::features.change = 1;
					return 1;
				}
			}
	}
	return 0;
}


#endif // AUDIO_INTERFACE
