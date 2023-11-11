/* -*- C -*- */

#ifndef CONTIKI_CONF_H
#define CONTIKI_CONF_H

/* include the project config */
#ifdef PROJECT_CONF_PATH
#include PROJECT_CONF_PATH
#endif /* PROJECT_CONF_PATH */
/*---------------------------------------------------------------------------*/
#include "sky-def.h"
#include "msp430-def.h"
/*---------------------------------------------------------------------------*/

/* Configure radio driver */
#ifndef NETSTACK_CONF_RADIO
#define NETSTACK_CONF_RADIO   fwpxtrx_driver
#endif /* NETSTACK_CONF_RADIO */

/* Save RAM through a smaller uIP buffer */
#ifndef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE		240
#endif

/* Platform-specific (H/W) AES implementation */
#ifndef AES_128_CONF
#define AES_128_CONF cc2420_aes_128_driver
#endif /* AES_128_CONF */
/*---------------------------------------------------------------------------*/
#include "msp430-conf.h"
/*---------------------------------------------------------------------------*/
#endif /* CONTIKI_CONF_H */
