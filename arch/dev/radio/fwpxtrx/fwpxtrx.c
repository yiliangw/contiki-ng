/*
 * Copyright (c) 2007, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/*
 * This code is almost device independent and should be easy to port.
 */

#include <string.h>

#include "contiki.h"
#include "sys/energest.h"

#include "dev/leds.h"
#include "dev/spi-legacy.h"
#include "dev/radio/fwpxtrx/fwpxtrx.h"
#include "dev/radio/fwpxtrx/fwpxtrx_const.h"

#include "net/packetbuf.h"
#include "net/netstack.h"

enum write_ram_order {
  /* Begin with writing the first given byte */
  WRITE_RAM_IN_ORDER,
  /* Begin with writing the last given byte */
  WRITE_RAM_REVERSE
};

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

#define DEBUG_LEDS DEBUG
#undef LEDS_ON
#undef LEDS_OFF
#if DEBUG_LEDS
#define LEDS_ON(x) leds_on(x)
#define LEDS_OFF(x) leds_off(x)
#else
#define LEDS_ON(x)
#define LEDS_OFF(x)
#endif

/* Conversion map between PA_LEVEL and output power in dBm
   (from table 9 in CC2420 specification).
*/
struct output_config {
  int8_t power;
  uint8_t config;
};

static const struct output_config output_power[] = {
  {  0, 31 }, /* 0xff */
  { -1, 27 }, /* 0xfb */
  { -3, 23 }, /* 0xf7 */
  { -5, 19 }, /* 0xf3 */
  { -7, 15 }, /* 0xef */
  {-10, 11 }, /* 0xeb */
  {-15,  7 }, /* 0xe7 */
  {-25,  3 }, /* 0xe3 */
};
#define OUTPUT_NUM (sizeof(output_power) / sizeof(struct output_config))
#define OUTPUT_POWER_MAX   0
#define OUTPUT_POWER_MIN -25

void cc2420_arch_init(void);

int cc2420_authority_level_of_sender;

volatile uint8_t cc2420_sfd_counter;
volatile uint16_t cc2420_sfd_start_time;
volatile uint16_t cc2420_sfd_end_time;

/*
 * The maximum number of bytes this driver can accept from the MAC layer for
 * transmission or will deliver to the MAC layer after reception. Includes
 * the MAC header and payload, but not the FCS.
 */
#define MAX_PAYLOAD_LEN (127 - CHECKSUM_LEN)
/*---------------------------------------------------------------------------*/
PROCESS(fwpxtrx_process, "fwpxtrx driver");
/*---------------------------------------------------------------------------*/

#define AUTOACK (1 << 4)
#define AUTOCRC (1 << 5)
#define ADR_DECODE (1 << 11)
#define RXFIFO_PROTECTION (1 << 9)
#define CORR_THR(n) (((n) & 0x1f) << 6)
#define FIFOP_THR(n) ((n) & 0x7f)
#define RXBPF_LOCUR (1 << 13);
#define TX_MODE (3 << 2)

int cc2420_on(void);
int cc2420_off(void);

static int cc2420_read(void *buf, unsigned short bufsize);

static int cc2420_prepare(const void *data, unsigned short len);
static int cc2420_transmit(unsigned short len);
static int cc2420_send(const void *data, unsigned short len);

static int cc2420_receiving_packet(void);
static int pending_packet(void);
static int get_cca_threshold(void);
static int cc2420_cca(void);
static uint16_t getreg(enum cc2420_register regname);

static void set_frame_filtering(uint8_t enable);
static void set_poll_mode(uint8_t enable);
static void set_send_on_cca(uint8_t enable);
static void set_auto_ack(uint8_t enable);

static void set_test_mode(uint8_t enable, uint8_t modulated);

signed char cc2420_last_rssi;
uint8_t cc2420_last_correlation;

static uint8_t receive_on;

static int tx_channel;
static int rx_channel;
static int rx_opcode;
static int rx_pkt_len;
static int rx_num;

/* pointer to a function */

static void (* input_callback)(void);

/* Are we currently in poll mode? */
static uint8_t volatile poll_mode = 0;
/* Do we perform a CCA before sending? */
static uint8_t send_on_cca = WITH_SEND_CCA;

static radio_result_t
get_value(radio_param_t param, radio_value_t *value)
{
  int i, v;

  if(!value) {
    return RADIO_RESULT_INVALID_VALUE;
  }
  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    if((getreg(CC2420_MDMCTRL1) & TX_MODE) & 0x08) {
      *value = RADIO_POWER_MODE_CARRIER_ON;
    } else {
      *value = receive_on ? RADIO_POWER_MODE_ON : RADIO_POWER_MODE_OFF;
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CHANNEL:
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_RX_CHANNEL:
    *value = rx_channel;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_OPCODE:
    *value = rx_opcode;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_NUM:
    *value = rx_num;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    *value = 0;
    if(getreg(CC2420_MDMCTRL0) & ADR_DECODE) {
      *value |= RADIO_RX_MODE_ADDRESS_FILTER;
    }
    if(getreg(CC2420_MDMCTRL0) & AUTOACK) {
      *value |= RADIO_RX_MODE_AUTOACK;
    }
    if(poll_mode) {
      *value |= RADIO_RX_MODE_POLL_MODE;
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    *value = 0;
    if(send_on_cca) {
      *value |= RADIO_TX_MODE_SEND_ON_CCA;
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    v = cc2420_get_txpower();
    *value = OUTPUT_POWER_MIN;
    /* Find the actual estimated output power in conversion table */
    for(i = 0; i < OUTPUT_NUM; i++) {
      if(v >= output_power[i].config) {
        *value = output_power[i].power;
        break;
      }
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CCA_THRESHOLD:
    *value = get_cca_threshold() + RSSI_OFFSET;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RSSI:
    /* Return the RSSI value in dBm */
    *value = cc2420_rssi();
    return RADIO_RESULT_OK;
  case RADIO_PARAM_LAST_RSSI:
    /* RSSI of the last packet received */
    *value = cc2420_last_rssi;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_LAST_LINK_QUALITY:
    /* LQI of the last packet received */
    *value = cc2420_last_correlation;
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MIN:
    *value = 11;
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MAX:
    *value = 26;
    return RADIO_RESULT_OK;
  case RADIO_CONST_TXPOWER_MIN:
    *value = OUTPUT_POWER_MIN;
    return RADIO_RESULT_OK;
  case RADIO_CONST_TXPOWER_MAX:
    *value = OUTPUT_POWER_MAX;
    return RADIO_RESULT_OK;
  case RADIO_CONST_MAX_PAYLOAD_LEN:
    *value = (radio_value_t)MAX_PAYLOAD_LEN;
    return RADIO_RESULT_OK;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}

static radio_result_t
set_value(radio_param_t param, radio_value_t value)
{
  int i;

  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    if(value == RADIO_POWER_MODE_ON) {
      cc2420_on();
      return RADIO_RESULT_OK;
    }
    if(value == RADIO_POWER_MODE_OFF) {
      cc2420_off();
      return RADIO_RESULT_OK;
    }
    if(value == RADIO_POWER_MODE_CARRIER_ON ||
       value == RADIO_POWER_MODE_CARRIER_OFF) {
      set_test_mode((value == RADIO_POWER_MODE_CARRIER_ON), 0);
      return RADIO_RESULT_OK;
    }
    return RADIO_RESULT_INVALID_VALUE;
  case RADIO_PARAM_CHANNEL:
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_TX_CHANNEL:
    if(value < 11 || value > 26) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    tx_channel = value;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    if(value & ~(RADIO_RX_MODE_ADDRESS_FILTER |
        RADIO_RX_MODE_AUTOACK | RADIO_RX_MODE_POLL_MODE)) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    set_frame_filtering((value & RADIO_RX_MODE_ADDRESS_FILTER) != 0);
    set_auto_ack((value & RADIO_RX_MODE_AUTOACK) != 0);
    set_poll_mode((value & RADIO_RX_MODE_POLL_MODE) != 0);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    if(value & ~(RADIO_TX_MODE_SEND_ON_CCA)) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    set_send_on_cca((value & RADIO_TX_MODE_SEND_ON_CCA) != 0);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    if(value < OUTPUT_POWER_MIN || value > OUTPUT_POWER_MAX) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    /* Find the closest higher PA_LEVEL for the desired output power */
    for(i = 1; i < OUTPUT_NUM; i++) {
      if(value > output_power[i].power) {
        break;
      }
    }
    cc2420_set_txpower(output_power[i - 1].config);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CCA_THRESHOLD:
    cc2420_set_cca_threshold(value - RSSI_OFFSET);
    return RADIO_RESULT_OK;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}

static radio_result_t
get_object(radio_param_t param, void *dest, size_t size)
{
  if(param == RADIO_PARAM_LAST_PACKET_TIMESTAMP) {
#if CC2420_CONF_SFD_TIMESTAMPS
    if(size != sizeof(rtimer_clock_t) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    *(rtimer_clock_t*)dest = cc2420_sfd_start_time;
    return RADIO_RESULT_OK;
#else
    return RADIO_RESULT_NOT_SUPPORTED;
#endif
  }
  return RADIO_RESULT_NOT_SUPPORTED;
}

static radio_result_t
set_object(radio_param_t param, const void *src, size_t size)
{
  switch (param) {
  case RADIO_PARAM_INPUT_CALLBACK:
    if(size != sizeof(input_callback) || !src) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    memcpy(&input_callback, src, size);
    return RADIO_RESULT_OK;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}

const struct radio_driver fwpxtrx_driver =
  {
    cc2420_init,
    cc2420_prepare,
    cc2420_transmit,
    cc2420_send,
    cc2420_read,
    cc2420_cca,
    cc2420_receiving_packet,
    pending_packet,
    cc2420_on,
    cc2420_off,
    get_value,
    set_value,
    get_object,
    set_object
  };

/*---------------------------------------------------------------------------*/
/* Sends a strobe */
static void
strobe(enum cc2420_register regname)
{
  CC2420_SPI_ENABLE();
  SPI_WRITE(regname);
  CC2420_SPI_DISABLE();
}
/*---------------------------------------------------------------------------*/
/* Reads a register */
static uint16_t
getreg(enum cc2420_register regname)
{
  uint16_t value;

  CC2420_SPI_ENABLE();
  SPI_WRITE(regname | 0x40);
  value = (uint8_t)SPI_RXBUF;
  SPI_TXBUF = 0;
  SPI_WAITFOREORx();
  value = SPI_RXBUF << 8;
  SPI_TXBUF = 0;
  SPI_WAITFOREORx();
  value |= SPI_RXBUF;
  CC2420_SPI_DISABLE();

  return value;
}
/*---------------------------------------------------------------------------*/
/**
 * Writes to a register.
 * Note: the SPI_WRITE(0) seems to be needed for getting the
 * write reg working on the Z1 / MSP430X platform
 */
static void
setreg(enum cc2420_register regname, uint16_t value)
{
  CC2420_SPI_ENABLE();
  SPI_WRITE_FAST(regname);
  SPI_WRITE_FAST((uint8_t) (value >> 8));
  SPI_WRITE_FAST((uint8_t) (value & 0xff));
  SPI_WAITFORTx_ENDED();
  SPI_WRITE(0);
  CC2420_SPI_DISABLE();
}
/*---------------------------------------------------------------------------*/
static void
read_ram(uint8_t *buffer, uint16_t adr, uint16_t count)
{
  uint8_t i;

  CC2420_SPI_ENABLE();
  SPI_WRITE(0x80 | ((adr) & 0x7f));
  SPI_WRITE((((adr) >> 1) & 0xc0) | 0x20);
  SPI_RXBUF;
  for(i = 0; i < count; i++) {
    SPI_READ(((uint8_t*) buffer)[i]);
  }
  CC2420_SPI_DISABLE();
}
/*---------------------------------------------------------------------------*/
/* Write to RAM in the CC2420 */
static void
write_ram(const uint8_t *buffer,
    uint16_t adr,
    uint16_t count,
    enum write_ram_order order)
{
  uint8_t i;

  CC2420_SPI_ENABLE();
  SPI_WRITE_FAST(0x80 | (adr & 0x7f));
  SPI_WRITE_FAST((adr >> 1) & 0xc0);
  if(order == WRITE_RAM_IN_ORDER) {
    for(i = 0; i < count; i++) {
      SPI_WRITE_FAST((buffer)[i]);
    }
  } else {
    for(i = count; i > 0; i--) {
      SPI_WRITE_FAST((buffer)[i - 1]);
    }
  }
  SPI_WAITFORTx_ENDED();
  CC2420_SPI_DISABLE();
}
/*---------------------------------------------------------------------------*/
static void
write_fifo_buf(const uint8_t *buffer, uint16_t count)
{
  uint8_t i;

  CC2420_SPI_ENABLE();
  SPI_WRITE_FAST(CC2420_TXFIFO);
  for(i = 0; i < count; i++) {
    SPI_WRITE_FAST((buffer)[i]);
  }
  SPI_WAITFORTx_ENDED();
  CC2420_SPI_DISABLE();
}
/*---------------------------------------------------------------------------*/
/* Returns the current status */
static uint8_t
get_status(void)
{
  uint8_t status;

  CC2420_SPI_ENABLE();
  SPI_WRITE(CC2420_SNOP);
  status = SPI_RXBUF;
  CC2420_SPI_DISABLE();

  return status;
}
/*---------------------------------------------------------------------------*/
static void
getrxdata(uint8_t *buffer, int count)
{
  uint8_t i;

  CC2420_SPI_ENABLE();
  SPI_WRITE(CC2420_RXFIFO | 0x40);
  (void) SPI_RXBUF;
  for(i = 0; i < count; i++) {
    SPI_READ(buffer[i]);
  }
  clock_delay(1);
  CC2420_SPI_DISABLE();
}
/*---------------------------------------------------------------------------*/
static void
flushrx(void)
{
  strobe(CC2420_SFLUSHRX);
}
/*---------------------------------------------------------------------------*/
static void
wait_for_status(uint8_t status_bit)
{
  rtimer_clock_t t0;
  t0 = RTIMER_NOW();
  while(!(get_status() & status_bit)
      && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (RTIMER_SECOND / 10)));
}
/*---------------------------------------------------------------------------*/
static void
wait_for_transmission(void)
{
  rtimer_clock_t t0;
  t0 = RTIMER_NOW();
  while((get_status() & BV(CC2420_TX_ACTIVE))
      && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (RTIMER_SECOND / 10)));
}
/*---------------------------------------------------------------------------*/
static void
on(void)
{
  if(!poll_mode) {
    CC2420_ENABLE_FIFOP_INT();
  }

  strobe(CC2420_SRXON);

  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  receive_on = 1;
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
  /*  PRINTF("off\n");*/
  receive_on = 0;

  /* Wait for transmission to end before turning radio off. */
  wait_for_transmission();

  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  strobe(CC2420_SRFOFF);
  if(!poll_mode) {
    CC2420_DISABLE_FIFOP_INT();
  }

  if(!CC2420_FIFOP_IS_1) {
    flushrx();
  }
}
/*---------------------------------------------------------------------------*/
static uint8_t locked, lock_on, lock_off;
#define GET_LOCK() locked++
static void RELEASE_LOCK(void) {
  if(locked == 1) {
    if(lock_on) {
      on();
      lock_on = 0;
    }
    if(lock_off) {
      off();
      lock_off = 0;
    }
  }
  locked--;
}
/*---------------------------------------------------------------------------*/
static void
init_security(void)
{
  /* only use key 0 */
  setreg(CC2420_SECCTRL0, 0);
  setreg(CC2420_SECCTRL1, 0);
}
/*---------------------------------------------------------------------------*/
static void
set_key(const uint8_t *key)
{
  GET_LOCK();

  write_ram(key, CC2420RAM_KEY0, 16, WRITE_RAM_REVERSE);

  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
static void
encrypt(uint8_t *plaintext_and_result)
{
  GET_LOCK();

  write_ram(plaintext_and_result,
      CC2420RAM_SABUF,
      16,
      WRITE_RAM_IN_ORDER);

  strobe(CC2420_SAES);
  while(get_status() & BV(CC2420_ENC_BUSY));

  read_ram(plaintext_and_result, CC2420RAM_SABUF, 16);

  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
const struct aes_128_driver cc2420_aes_128_driver = {
  set_key,
  encrypt
};
/*---------------------------------------------------------------------------*/
static void
set_txpower(uint8_t power)
{
  uint16_t reg;

  reg = getreg(CC2420_TXCTRL);
  reg = (reg & 0xffe0) | (power & 0x1f);
  setreg(CC2420_TXCTRL, reg);
}
/*---------------------------------------------------------------------------*/
int
cc2420_init(void)
{
  uint16_t reg;
  {
    int s = splhigh();
    cc2420_arch_init();		/* Initalize ports and SPI. */
    CC2420_DISABLE_FIFOP_INT();
    CC2420_FIFOP_INT_INIT();
    splx(s);
  }

  /* Turn on voltage regulator and reset. */
  SET_VREG_ACTIVE();
  clock_delay(250);
  SET_RESET_ACTIVE();
  clock_delay(127);
  SET_RESET_INACTIVE();
  clock_delay(125);


  /* Turn on the crystal oscillator. */
  strobe(CC2420_SXOSCON);
  /* And wait until it stabilizes */
  wait_for_status(BV(CC2420_XOSC16M_STABLE));

  /* Set auto-ack and frame filtering */
  set_auto_ack(CC2420_CONF_AUTOACK);
  set_frame_filtering(CC2420_CONF_AUTOACK);

  /* Enabling CRC in hardware; this is required by AUTOACK anyway
     and provides us with RSSI and link quality indication (LQI)
     information. */
  reg = getreg(CC2420_MDMCTRL0);
  reg |= AUTOCRC;
  setreg(CC2420_MDMCTRL0, reg);

  /* Set transmission turnaround time to the lower setting (8 symbols
     = 0.128 ms) instead of the default (12 symbols = 0.192 ms). */
  /*  reg = getreg(CC2420_TXCTRL);
  reg &= ~(1 << 13);
  setreg(CC2420_TXCTRL, reg);*/


  /* Change default values as recomended in the data sheet, */
  /* correlation threshold = 20, RX bandpass filter = 1.3uA. */
  setreg(CC2420_MDMCTRL1, CORR_THR(20));
  reg = getreg(CC2420_RXCTRL1);
  reg |= RXBPF_LOCUR;
  setreg(CC2420_RXCTRL1, reg);

  /* Set the FIFOP threshold to maximum. */
  setreg(CC2420_IOCFG0, FIFOP_THR(127));

  init_security();

  cc2420_set_pan_addr(0xffff, 0x0000, NULL);
  tx_channel = IEEE802154_DEFAULT_CHANNEL;
  cc2420_set_cca_threshold(CC2420_CONF_CCA_THRESH);

  flushrx();

  set_poll_mode(0);

  process_start(&fwpxtrx_process, NULL);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_transmit(unsigned short payload_len)
{
  if(payload_len > MAX_PAYLOAD_LEN) {
    return RADIO_TX_ERR;
  }

  GET_LOCK();

  strobe(CC2420_STXON);
  
  RELEASE_LOCK();

  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_prepare(const void *payload, unsigned short payload_len)
{
  uint8_t total_len;
  uint8_t channel;

  if(payload_len > MAX_PAYLOAD_LEN) {
    return RADIO_TX_ERR;
  }

  GET_LOCK();

  PRINTF("cc2420: sending %d bytes\n", payload_len);

  /* Wait for any previous transmission to finish. */
  /*  while(status() & BV(CC2420_TX_ACTIVE));*/

  /* Write packet to TX FIFO. */
  strobe(CC2420_SFLUSHTX);

  total_len = payload_len;
  channel = tx_channel;
  write_fifo_buf(&channel, 1);
  write_fifo_buf(&total_len, 1);
  write_fifo_buf(payload, payload_len);

  RELEASE_LOCK();
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_send(const void *payload, unsigned short payload_len)
{
  cc2420_prepare(payload, payload_len);
  return cc2420_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
int
cc2420_off(void)
{
  /* Don't do anything if we are already turned off. */
  if(receive_on == 0) {
    return 1;
  }

  /* If we are called when the driver is locked, we indicate that the
     radio should be turned off when the lock is unlocked. */
  if(locked) {
    /*    printf("Off when locked (%d)\n", locked);*/
    lock_off = 1;
    return 1;
  }

  GET_LOCK();
  /* If we are currently receiving a packet (indicated by SFD == 1),
     we don't actually switch the radio off now, but signal that the
     driver should switch off the radio once the packet has been
     received and processed, by setting the 'lock_off' variable. */
  if(get_status() & BV(CC2420_TX_ACTIVE)) {
    lock_off = 1;
  } else {
    off();
  }
  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
int
cc2420_on(void)
{
  if(receive_on) {
    return 1;
  }
  if(locked) {
    lock_on = 1;
    return 1;
  }

  GET_LOCK();
  on();
  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_pan_addr(unsigned pan,
                    unsigned addr,
                    const uint8_t *ieee_addr)
{
  GET_LOCK();

  write_ram((uint8_t *) &pan, CC2420RAM_PANID, 2, WRITE_RAM_IN_ORDER);
  write_ram((uint8_t *) &addr, CC2420RAM_SHORTADDR, 2, WRITE_RAM_IN_ORDER);

  if(ieee_addr != NULL) {
    write_ram(ieee_addr, CC2420RAM_IEEEADDR, 8, WRITE_RAM_REVERSE);
  }
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
/*
 * Interrupt leaves frame intact in FIFO.
 */
int
cc2420_interrupt(void)
{
  CC2420_CLEAR_FIFOP_INT();
  process_poll(&fwpxtrx_process);

  return 1;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(fwpxtrx_process, ev, data)
{
  int len;
  PROCESS_BEGIN();

  PRINTF("cc2420_process: started\n");

  while(1) {
    PROCESS_YIELD_UNTIL(!poll_mode && ev == PROCESS_EVENT_POLL);

    PRINTF("cc2420_process: calling receiver callback\n");

    packetbuf_clear();
    len = cc2420_read(packetbuf_dataptr(), PACKETBUF_SIZE);

    packetbuf_set_datalen(len);

    if (input_callback) {
      input_callback();
    } else {
      NETSTACK_MAC.input();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static int
cc2420_read(void *buf, unsigned short bufsize)
{
  uint8_t opcode, channel;

  if(!CC2420_FIFOP_IS_1) {
    return 0;
  }

  GET_LOCK();

  getrxdata(&opcode, 1);
  getrxdata(&channel, 1);

  rx_opcode = opcode;
  rx_channel = channel;
  rx_pkt_len = 0;

  switch(opcode) {
    case RADIO_RX_OPCODE_PKT: {
      uint8_t len;
      getrxdata(&len, 1);
      if(len > CC2420_MAX_PACKET_LEN || len > bufsize) {
        /* Packet too long */
      } else {
        rx_pkt_len = len;
        getrxdata(buf, len);        
      }
    }
    break;
    case RADIO_RX_OPCODE_PKTNUM: {
      uint8_t num;
      getrxdata(&num, 1);
      rx_num = num;
    }
    break;
    default:
      PRINTF("Unknown RX opcode\n");
  }

  PRINTF("cc2420_read: opcode %d channel %d\n", opcode, channel);

  RELEASE_LOCK();
  return rx_pkt_len;
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_txpower(uint8_t power)
{
  GET_LOCK();
  set_txpower(power);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
int
cc2420_get_txpower(void)
{
  int power;
  GET_LOCK();
  power = (int)(getreg(CC2420_TXCTRL) & 0x001f);
  RELEASE_LOCK();
  return power;
}
/*---------------------------------------------------------------------------*/
int
cc2420_rssi(void)
{
  int rssi;
  int radio_was_off = 0;

  if(locked) {
    return 0;
  }

  GET_LOCK();

  if(!receive_on) {
    radio_was_off = 1;
    cc2420_on();
  }
  wait_for_status(BV(CC2420_RSSI_VALID));

  rssi = (int)((signed char) getreg(CC2420_RSSI));
  rssi += RSSI_OFFSET;

  if(radio_was_off) {
    cc2420_off();
  }
  RELEASE_LOCK();
  return rssi;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_cca(void)
{
  int cca;
  int radio_was_off = 0;

  /* If the radio is locked by an underlying thread (because we are
     being invoked through an interrupt), we preted that the coast is
     clear (i.e., no packet is currently being transmitted by a
     neighbor). */
  if(locked) {
    return 1;
  }

  GET_LOCK();
  if(!receive_on) {
    radio_was_off = 1;
    cc2420_on();
  }

  /* Make sure that the radio really got turned on. */
  if(!receive_on) {
    RELEASE_LOCK();
    if(radio_was_off) {
      cc2420_off();
    }
    return 1;
  }

  wait_for_status(BV(CC2420_RSSI_VALID));

  cca = CC2420_CCA_IS_1;

  if(radio_was_off) {
    cc2420_off();
  }
  RELEASE_LOCK();
  return cca;
}
/*---------------------------------------------------------------------------*/
int
cc2420_receiving_packet(void)
{
  return CC2420_SFD_IS_1;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  return CC2420_FIFOP_IS_1;
}
/*---------------------------------------------------------------------------*/
static int
get_cca_threshold(void)
{
  int value;

  GET_LOCK();
  value = (int8_t)(getreg(CC2420_RSSI) >> 8);
  RELEASE_LOCK();
  return value;
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_cca_threshold(int value)
{
  uint16_t shifted = value << 8;
  GET_LOCK();
  setreg(CC2420_RSSI, shifted);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
/* Set or unset frame autoack */
static void
set_auto_ack(uint8_t enable)
{
  GET_LOCK();

  uint16_t reg = getreg(CC2420_MDMCTRL0);
  if(enable) {
    reg |= AUTOACK;
  } else {
    reg &= ~(AUTOACK);
  }

  setreg(CC2420_MDMCTRL0, reg);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
/* Set or unset frame filtering */
static void
set_frame_filtering(uint8_t enable)
{
  GET_LOCK();

  /* Turn on/off address decoding. */
  uint16_t reg = getreg(CC2420_MDMCTRL0);
  if(enable) {
    reg |= ADR_DECODE;
  } else {
    reg &= ~(ADR_DECODE);
  }

  setreg(CC2420_MDMCTRL0, reg);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
/* Enable or disable radio interrupts (both FIFOP and SFD timer capture) */
static void
set_poll_mode(uint8_t enable)
{
  GET_LOCK();
  poll_mode = enable;
  if(enable) {
    /* Disable FIFOP interrupt */
    CC2420_CLEAR_FIFOP_INT();
    CC2420_DISABLE_FIFOP_INT();
  } else {
    /* Initialize and enable FIFOP interrupt */
    CC2420_FIFOP_INT_INIT();
    CC2420_ENABLE_FIFOP_INT();
    CC2420_CLEAR_FIFOP_INT();
  }
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
/* Enable or disable CCA before sending */
static void
set_send_on_cca(uint8_t enable)
{
  send_on_cca = enable;
}
/*---------------------------------------------------------------------------*/
/* Enable or disable radio test mode emmiting modulated or unmodulated
 * (carrier) signal. See datasheet page 55.
 */
static uint16_t prev_MDMCTRL1, prev_DACTST;
static uint8_t was_on;

static void
set_test_mode(uint8_t enable, uint8_t modulated)
{
  radio_value_t mode;
  get_value(RADIO_PARAM_POWER_MODE, &mode);

  if(enable) {
    if(mode == RADIO_POWER_MODE_CARRIER_ON) {
      return;
    }
    was_on = (mode == RADIO_POWER_MODE_ON);
    off();
    prev_MDMCTRL1 = getreg(CC2420_MDMCTRL1);
    setreg(CC2420_MDMCTRL1, 0x050C);
    if(!modulated) {
      prev_DACTST = getreg(CC2420_DACTST);
      setreg(CC2420_DACTST, 0x1800);
    }
    /* actually starts the test mode */
    strobe(CC2420_STXON);
  } else {
    if(mode != RADIO_POWER_MODE_CARRIER_ON) {
      return;
    }
    strobe(CC2420_SRFOFF);
    if(!modulated) {
      setreg(CC2420_DACTST, prev_DACTST);
    }
    setreg(CC2420_MDMCTRL1, prev_MDMCTRL1);
    /* actually stops the carrier */
    if(was_on) {
      on();
    }
  }
}
/*---------------------------------------------------------------------------*/
