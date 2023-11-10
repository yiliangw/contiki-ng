/**
 * \file
 *         A MAC layer that direclty relays the data.
 */

 #include "net/mac/rawmac/rawmac.h"
 #include "net/netstack.h"
 #include "net/packetbuf.h"
 #include <string.h>

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "RawMAC"
#define LOG_LEVEL LOG_LEVEL_MAC

/* If the packet starts with a random character, then it may not be 
   delivered (at least in Cooja) in some cases (e.g., staring with 'a'). */
static char *hdr = " ";

/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  const size_t hdr_len = strlen(hdr);
  /* Create the frame */
  if (packetbuf_hdralloc(hdr_len) < 0) {
    LOG_ERR("failed to create the frame %d\n", packetbuf_datalen());
    return;
  }
  memcpy(packetbuf_hdrptr(), hdr, strlen(hdr));

  NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen());
  if (sent)
    sent(ptr, MAC_TX_OK, 1);
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
  const size_t hdr_len = strlen(hdr);
  /* Remove the frame */
  if (packetbuf_hdrreduce(hdr_len) < 0) {
    LOG_ERR("failed to parse %u\n", packetbuf_datalen());
    return;
  }
  NETSTACK_NETWORK.input();
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
  return NETSTACK_RADIO.off();
}
/*---------------------------------------------------------------------------*/
static int
max_payload(void)
{
  int framer_hdrlen;
  radio_value_t max_radio_payload_len;
  radio_result_t res;

  framer_hdrlen = NETSTACK_FRAMER.length();

  res = NETSTACK_RADIO.get_value(RADIO_CONST_MAX_PAYLOAD_LEN,
                                 &max_radio_payload_len);
  
  if(res == RADIO_RESULT_NOT_SUPPORTED) {
    LOG_ERR("Failed to retrieve max radio driver payload length\n");
    return 0;
  }
                                 
  return MIN(max_radio_payload_len, PACKETBUF_SIZE) - framer_hdrlen;
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  radio_value_t radio_max_payload_len;

  /* Check that the radio can correctly report its max supported payload */
  if(NETSTACK_RADIO.get_value(RADIO_CONST_MAX_PAYLOAD_LEN, &radio_max_payload_len) != RADIO_RESULT_OK) {
    LOG_ERR("! radio does not support getting RADIO_CONST_MAX_PAYLOAD_LEN. Abort init.\n");
    return;
  }
}
/*---------------------------------------------------------------------------*/
const struct mac_driver rawmac_driver = {
  "RawMAC",
  init,
  send_packet,
  packet_input,
  on,
  off,
  max_payload,
};
/*---------------------------------------------------------------------------*/
