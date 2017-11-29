/***********************************************************************************************//**


#ifndef GENERATION_DONE
#error You must run generate first!
#endif

/* Board headers */
#include "boards.h"
#include "ble-configuration.h"
#include "board_features.h"


/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include "aat.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_rtcc.h"

#include "retargetserial.h"
#include "retargetserialconfig.h"
#include "sleep.h"


#ifdef FEATURE_BOARD_DETECTED
#include "bspconfig.h"
#include "pti.h"
#endif

/* Device initialization header */
#include "InitDevice.h"

#ifdef FEATURE_SPI_FLASH
#include "em_usart.h"
#include "mx25flash_spi.h"
#endif /* FEATURE_SPI_FLASH */


/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/
#define CONN_INTERVAL_MIN  48  //12.5ms
#define CONN_INTERVAL_MAX  48  //12.5ms
#define CONN_SLAVE_LATENCY 10   //no latency
#define CONN_TIMEOUT       32  //100ms

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif


#ifdef FEATURE_PTI_SUPPORT
static const RADIO_PTIInit_t ptiInit = RADIO_PTI_INIT;
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];
/* Gecko configuration parameters (see gecko_configuration.h) */
static const gecko_configuration_t config = {
  .config_flags=0,
  .sleep.flags=SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections=MAX_CONNECTIONS,
  .bluetooth.heap=bluetooth_stack_heap,
  .bluetooth.heap_size=sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb=&bg_gattdb_data,
  .ota.flags=0,
  .ota.device_name_len=3,
  .ota.device_name_ptr="OTA",
  #ifdef FEATURE_PTI_SUPPORT
  .pti = &ptiInit,
  #endif
};

/* Flag for indicating DFU Reset must be performed */
uint32_t time1, time2, timediff;
uint8 myData[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
uint8_t isNotify = 0;


unsigned char adv_data[28] =
  {0x02,0x01,0x06,
   0x11,0x07,
   0x31,0x59, 0x69,0x3f,0x1e,0x1f,0x43,0x9c,
   0xa3,0x88,0xd5,0x49,0xcc,0x26,0x19,0x38,
   0x06,0x09,
   's','l','a','v','e'};

/* push button generates external signal event for the stacks */
void handle_button_push(uint8_t pin) {
	gecko_external_signal((uint32) pin);
}

void main(void)
{

#ifdef FEATURE_SPI_FLASH
  /* Put the SPI flash into Deep Power Down mode for those radio boards where it is available */
  MX25_init();
  MX25_DP();
  /* We must disable SPI communication */
  USART_Reset(USART1);

#endif /* FEATURE_SPI_FLASH */

  /* Initialize peripherals */
  enter_DefaultMode_from_RESET();

  /* Initialize stack */
  gecko_init(&config);

  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;
    
    /* Check for stack event. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {

      /* This boot event is generated when the system boots up after reset.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:
    	  gecko_cmd_le_gap_set_conn_parameters(CONN_INTERVAL_MIN,
    	  	CONN_INTERVAL_MAX, CONN_SLAVE_LATENCY, CONN_TIMEOUT);
        /* Set advertising parameters. 100ms advertisement interval. All channels used.
         * The first two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6). The third parameter '7' sets advertising on all channels. */
        gecko_cmd_le_gap_set_adv_parameters(160,160,7);

        gecko_cmd_le_gap_set_adv_data(0, 28, adv_data);

        /* Start general advertising and enable connections. */
        gecko_cmd_le_gap_set_mode(le_gap_user_data, le_gap_undirected_connectable);
        GPIO_PinOutSet(LED_G_PORT,LED_G_PIN);
        break;

      case gecko_evt_le_connection_opened_id:

    	  GPIO_PinOutClear(LED_G_PORT,LED_G_PIN);

        break;

      case gecko_evt_le_connection_closed_id:


          gecko_cmd_le_gap_set_adv_data(0, 28, adv_data);
		  
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_set_mode(le_gap_user_data, le_gap_undirected_connectable);

        break;

case gecko_evt_gatt_server_user_write_request_id:

              if(evt->data.evt_gatt_server_user_write_request.characteristic==gattdb_custom_characteristic)
              {
                // Send response to Write Request
                gecko_cmd_gatt_server_send_user_write_response(
                  evt->data.evt_gatt_server_user_write_request.connection,
                  gattdb_custom_characteristic,
                  bg_err_success);

              }
              break;


      default:
        break;
    }
  }
}


/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
