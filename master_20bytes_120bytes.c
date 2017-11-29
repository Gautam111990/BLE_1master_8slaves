
#ifndef GENERATION_DONE
#error You must run generate first!
#endif

#define RETARGET_VCOM

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
#include "em_rtcc.h"
#ifdef FEATURE_BOARD_DETECTED
#include "bspconfig.h"
#include "pti.h"
#endif

/* Device initialization header */
#include "InitDevice.h"

#ifdef FEATURE_SPI_FLASH
#include "em_usart.h"
#include "mx25flash_spi.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "stdio.h"
#include "retargetserial.h"
#include "gpiointerrupt.h"

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

/* connection parameters */
#define CONN_INTERVAL_MIN  10  //12.5ms
#define CONN_INTERVAL_MAX  10  //12.5ms
#define CONN_SLAVE_LATENCY 0   //no latency
#define CONN_TIMEOUT       10  //100ms

#define scan_interval_min  100 //200ms
#define scan_interval_max  100 //200ms
#define passive 0

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 8
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

#ifdef FEATURE_PTI_SUPPORT
static const RADIO_PTIInit_t ptiInit = RADIO_PTI_INIT;
#endif

		/*for (a=2*CONN_INTERVAL_MAX; a < CONN_TIMEOUT; a++)
		 {
		 }*/
		/* Gecko configuration parameters (see gecko_configuration.h) */
		static const gecko_configuration_t config = {
			.config_flags=0,
			.sleep.flags=0, //SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
		.bluetooth.max_connections=MAX_CONNECTIONS,
		.bluetooth.heap=bluetooth_stack_heap,
		.bluetooth.heap_size=sizeof(bluetooth_stack_heap),
		.gattdb=&bg_gattdb_data,
		.ota.flags=0,
		.ota.device_name_len=3,
		.ota.device_name_ptr="OTA",
#ifdef FEATURE_PTI_SUPPORT
		.pti = &ptiInit,
#endif
	};

	/* the UUID of the custom service we want to use */
	/*const char custom_service_uuid[16] =
	{	0x82,0x77, 0xe8,0x56,0x18,0xc9,0x4a,0x07,
		0x96,0xa8,0xb7,0x54,0x2a,0xa9,0x91,0x42};*/

	const char custom_service_uuid[16] =
	 {0x31,0x59, 0x69,0x3f,0x1e,0x1f,0x43,0x9c,
   0xa3,0x88,0xd5,0x49,0xcc,0x26,0x19,0x38};
	/* and its little endian version */
	char custom_service_uuid_le[16];

	/* the UUID of the custom characteristic we want to write to */
	/*const char custom_characteristic_uuid[16] =
	{	0x13, 0x79, 0x7d, 0x01, 0x1c, 0xc0, 0x4b, 0x7b,
		0xa4, 0xfd, 0x62, 0xe9, 0x9b, 0x39, 0x24, 0x72};*/

	const char custom_characteristic_uuid[16] =
	 {0x82, 0x6b, 0x70, 0x9f, 0x37, 0x3d, 0x4c, 0xe8,
	0x94, 0x98, 0x86, 0xe1,0xc1,0x08,0x9a,0x31};
//and its little endian version
	char custom_characteristic_uuid_le[16];

//const char device_name[5] = {'s','l','a','v','e'};

	/* connection handling variables for multiple slaves */
	uint8 connection_handle_table[MAX_CONNECTIONS];
	uint16 characteristic_handle_table[MAX_CONNECTIONS];
	uint32_t time1[MAX_CONNECTIONS]; 
	uint32_t time2[MAX_CONNECTIONS];
	uint8 active_connections_num;
	uint8 write_rsps_needed,writes_completed;
	uint8 notif_needed, notif_completed;
	uint32_t connection_packLost_table[MAX_CONNECTIONS] = {0};
	uint32_t connection_packSent_table[MAX_CONNECTIONS] = {0};

uint32_t t1, t2, t3, timediff, nowtime;
uint8 write_started = 0;
uint8 PB0_pressed = 0;
long packet_write = 0;
long packet_notif = 0;
int conDropCount = 0;
//uint32_t packet_write = 0;
long packet_sent = 0;
//uint32_t packet_write_rsp = 0;
uint32_t packet_ind = 0;
uint32_t packet_ind_conf = 0;
long packet_lost1 = 0;
uint32_t packet_lost2 = 0;
uint32_t packet_lost3 = 0;
//uint16 conn = 3;
//float x;
uint8 table_index;


typedef struct diag_pack_t{
int timeStamp;
//double packid;
//double charHandler;
//double data;
uint8 myData[16];
uint8 newData[100];
};
struct diag_pack_t  diagPack;
struct diag_pack_t* pDiagPack;

/* find the index of a given connection in the connection handle table */
uint8 find_connection_handle(uint8 connection) {
	uint8 c;
	for (c = 0; c < active_connections_num; c++) {
		if (connection_handle_table[c] == connection) {
			return c;
		}
	}

	return 0xFF;
}

/* push button generates external signal event for the stacks */
void handle_button_push(uint8_t pin) {
	gecko_external_signal((uint32) pin);
}

void loop_write() {
	uint8 i;
	write_rsps_needed = 0;
	writes_completed = 0;
	write_started = 1;
	notif_completed = 0;
	notif_needed = 0;
	diagPack.timeStamp = RTCC_CounterGet();
	/* issue write commands to all slaves */
	for (i = 0; i < active_connections_num; i++) {
		
		gecko_cmd_gatt_write_characteristic_value_without_response(connection_handle_table[i],
				characteristic_handle_table[i], sizeof(diagPack), (uint8_t*)&diagPack);
		notif_needed ++;
		packet_write++;
	}
}

void main() {
	uint8_t ad_field_length, ad_field_type;

	uint8 i;

	diagPack.myData[0] = 0;
	diagPack.myData[1] = 1;
	diagPack.myData[2] = 2;
	diagPack.myData[3] = 3;
	diagPack.myData[4] = 4;
	diagPack.myData[5] = 5;
	diagPack.myData[6] = 6;
	diagPack.myData[7] = 7;
	diagPack.myData[8] = 8;
	diagPack.myData[9] = 9;
	diagPack.myData[10] = 10;
	diagPack.myData[11] = 11;
	diagPack.myData[12] = 12;
	diagPack.myData[13] = 13;
	diagPack.myData[14] = 14;
	diagPack.myData[15] = 15;

	/* generating little endian UUIDs */
	for (i = 0; i < 16; i++)
		custom_service_uuid_le[i] = custom_service_uuid[15 - i];
	for (i = 0; i < 16; i++)
		custom_characteristic_uuid_le[i] = custom_characteristic_uuid[15 - i];

#ifdef FEATURE_SPI_FLASH
	/* Put the SPI flash into Deep Power Down mode for those radio boards where it is available */
	MX25_init();
	MX25_DP();
	/* We must disable SPI communication */
	USART_Reset(USART1);

#endif /* FEATURE_SPI_FLASH */

	/* Initialize peripherals */
	enter_DefaultMode_from_RESET();

	GPIOINT_Init();

	RETARGET_SerialInit();

	/* configuring push buttons PB0 and PB1 on the WSTK to generate interrupt */
	GPIO_ExtIntConfig(gpioPortF, 6, 0, false, true, true);
	GPIOINT_CallbackRegister(6, handle_button_push);
	GPIO_ExtIntConfig(gpioPortF, 7, 1, false, true, true);
	GPIOINT_CallbackRegister(7, handle_button_push);

	/* Initialize stack */
	gecko_init(&config);

	while (1) {

		struct gecko_cmd_packet* evt;

		/* Check for stack event. */
		evt = gecko_wait_event();

		/* Handle events */
		switch (BGLIB_MSG_ID(evt->header)) {

		/* This boot event is generated when the system boots up after reset. */
		case gecko_evt_system_boot_id:

			printf("\r\nBLE Central started\n\r");
			active_connections_num = 0;
			gecko_cmd_system_set_tx_power(80); //+8 dBm
			gecko_cmd_le_gap_set_scan_parameters(scan_interval_min,
					scan_interval_max, passive);
			gecko_cmd_le_gap_set_conn_parameters(CONN_INTERVAL_MIN,
					CONN_INTERVAL_MAX, CONN_SLAVE_LATENCY, CONN_TIMEOUT);
			/* start scanning for slaves */
			gecko_cmd_le_gap_discover(le_gap_discover_generic);
			printf("scanning...\r\n");

			break;

			/* This event is generated when an advertisement packet or a scan response is received from a slave */
		case gecko_evt_le_gap_scan_response_id:

			/* advertisement packet */
			if (evt->data.evt_le_gap_scan_response.packet_type == 0) {
				i = 0;
				/* parse advertisement packet */
				while (i < evt->data.evt_le_gap_scan_response.data.len) {
					ad_field_length =
							evt->data.evt_le_gap_scan_response.data.data[i];
					ad_field_type =
							evt->data.evt_le_gap_scan_response.data.data[i + 1];

					/* partial ($02) or complete ($03) list of 16-bit UUIDs */
					if (ad_field_type == 0x02 || ad_field_type == 0x03) {
						// handle partial ($02) or complete ($03) list of 16-bit UUIDs
					}

					/* complete local name */
					if (ad_field_type == 0x09) {
						// handle complete local name
					}

					/* complete list of 128bit uuids */
					if (ad_field_type == 0x07) {
						/* compare UUID to our custom service UUID */
						if (memcmp(
								&evt->data.evt_le_gap_scan_response.data.data[i
										+ 2], custom_service_uuid, 16) == 0) {
							printf("slave found\r\n");
							/* stop scanning */
							gecko_cmd_le_gap_end_procedure();
							if (active_connections_num < MAX_CONNECTIONS) {
								/* open connection */
								gecko_cmd_le_gap_open(
										evt->data.evt_le_gap_scan_response.address,
										evt->data.evt_le_gap_scan_response.address_type);
							}
							break;
						}
					}

					/* advance to the next AD struct */
					i = i + ad_field_length + 1;
				}
			}
			break;

			/* this event is generated when connection parameters are set */
		case gecko_evt_le_connection_parameters_id:

			printf("conn. params for #%d: %.1fms/%d/%dms\r\n",
					find_connection_handle(
							evt->data.evt_le_connection_parameters.connection),
					evt->data.evt_le_connection_parameters.interval*1.25,
					evt->data.evt_le_connection_parameters.latency,
					evt->data.evt_le_connection_parameters.timeout*10);
			break;

			/* this event is generated when a new connection is established */
		case gecko_evt_le_connection_opened_id:

			/* add handle to the connection handle table */
			connection_handle_table[active_connections_num] =
					evt->data.evt_le_connection_opened.connection;
			printf("connection #%d established\r\n", active_connections_num);
			active_connections_num++;

			printf("Discovering services...\n\r");
			/* discover custom service on the slave device */
			gecko_cmd_gatt_discover_primary_services_by_uuid(
					evt->data.evt_le_connection_opened.connection, 16,
					(const uint8*) custom_service_uuid_le);

			break;

			/* this event is generated when a connection is dropped */
		case gecko_evt_le_connection_closed_id:

			table_index = find_connection_handle(
					evt->data.evt_le_connection_closed.connection);
            
            conDropCount++;
			
			printf("connection #%d dropped %d time, reason: %x\n", table_index,
					conDropCount,
					evt->data.evt_le_connection_closed.reason);

			if (active_connections_num > 1) {
				active_connections_num--;
			}

			/* remove connection handle from the connection handle table */
			for (i = table_index; i < active_connections_num; i++) {
				connection_handle_table[i] = connection_handle_table[i + 1];
				characteristic_handle_table[i] = characteristic_handle_table[i + 1];
			}

			/* start scanning again*/
		    gecko_cmd_le_gap_set_scan_parameters( scan_interval_min, scan_interval_max, passive);
			 gecko_cmd_le_gap_discover(le_gap_discover_generic);
			 printf("scanning...\r\n");

			/*if (PB0_pressed == 1) {
				loop_write();
			}*/
			break;

			/* this event is generated when a new service is discovered */
		case gecko_evt_gatt_service_id:

			/* compare UUID to the UUID of our custom service */
			if (memcmp(evt->data.evt_gatt_service.uuid.data,
					custom_service_uuid_le, 16) == 0) {

				printf("custom service discovered\n\r");

				/* discover custom characteristic */
				gecko_cmd_gatt_discover_characteristics_by_uuid(
						evt->data.evt_gatt_service.connection,
						evt->data.evt_gatt_service.service, 16,
						(const uint8*) custom_characteristic_uuid_le);
			}

			break;

			/* this event is generated when a new characteristic is discovered */
		case gecko_evt_gatt_characteristic_id:

			table_index = find_connection_handle(
					evt->data.evt_gatt_characteristic.connection);

			if (table_index != 0xFF) {
				/* put characteristic handle into the characteristic handle table      *
				 * to the same index where the connection handle of this connection is */
				characteristic_handle_table[table_index] =
						evt->data.evt_gatt_characteristic.characteristic;
			}

			printf("custom characteristic discovered\r\n");

			/* stop discovering */
			gecko_cmd_le_gap_end_procedure();

			if (active_connections_num < MAX_CONNECTIONS) {
				/* start scanning againg */
				gecko_cmd_le_gap_set_scan_parameters(scan_interval_min,
						scan_interval_max, passive);
				gecko_cmd_le_gap_discover(le_gap_discover_generic);
				printf("scanning...\r\n");
			}


			if(PB0_pressed == 1 && active_connections_num == MAX_CONNECTIONS){

				PB0_pressed = 0;
				gecko_cmd_hardware_set_soft_timer(4000, 110, 1);

			}
			break;



		case gecko_evt_gatt_characteristic_value_id:

			if(write_started && evt->data.evt_gatt_characteristic_value.att_opcode == gatt_handle_value_notification){

			printf("-");
			notif_completed++;
			packet_notif++;
			if(notif_needed == notif_completed){
				 pDiagPack = (struct diag_pack_t*) evt->data.evt_gatt_characteristic_value.value.data;

				               nowtime = RTCC_CounterGet();

				               t1 = nowtime - pDiagPack->timeStamp;

				               printf("t1 %d ms \n\r", t1*1000/32768);

				               packet_sent ++;

				               if (t1*1000/32768 >= 100) {

				               packet_lost1++;

				                }
				            gecko_cmd_hardware_set_soft_timer(1000,20,1);
				           }
	  	} 

		/*if(PB0_pressed == 1){ //&& active_connections_num == MAX_CONNECTIONS){

              loop_write();

		  	}*/
			
     break;

			/* this event is triggered by gecko_external_signal() *
			 * in our case when a button is pushed                */
		case gecko_evt_system_external_signal_id:

			/* if PB1 is pushed start writing to slaves */
			if (evt->data.evt_system_external_signal.extsignals == 7) {

				loop_write();

		}
			/* if PB0 is pushed stop scanning and update connection parameters. */
			/* do this when all slaves are connected */
			if (evt->data.evt_system_external_signal.extsignals == 6) {
			//	printf("PB0 pressed\r\n");
				if (PB0_pressed == 0) {
					/* stop scanning */
					printf("stop scanning.\r\n");

					gecko_cmd_le_gap_end_procedure();
					for (i = 0; i < active_connections_num; i++) {
						gecko_cmd_le_connection_set_parameters(
								connection_handle_table[i],
								CONN_INTERVAL_MIN,
								CONN_INTERVAL_MAX,
								CONN_SLAVE_LATENCY,
								CONN_TIMEOUT);
						gecko_cmd_gatt_set_characteristic_notification(
								connection_handle_table[i],
								characteristic_handle_table[i],
								gatt_notification);

						if(conDropCount > 0)
						gecko_cmd_hardware_set_soft_timer(6000, 111, 1);

					}
					PB0_pressed = 1;
				} else {
					PB0_pressed = 2;
				    printf("Total number of t1 : %d \n\r",packet_sent);
				    printf("Total packets sent : %d \n\r",packet_write);
				    printf("Total packets received : %d \n\r",packet_notif);
				    printf("Total connection drop : %d \n\r",conDropCount);
					printf("packet lost t1 values more than 100ms : %ld\n",packet_lost1);
				}

			}
			break;

		case gecko_evt_hardware_soft_timer_id:
do{
	if(evt->data.evt_hardware_soft_timer.handle == 20){

	    	gecko_external_signal(7);
}
}while(PB0_pressed == 2);

		switch(evt->data.evt_hardware_soft_timer.handle){
				case 110: 

					gecko_external_signal(6);
				
				break; 
				case 111:

					gecko_external_signal(7);
				
				break; 
				default:

				break; 
			}

		break;	


		default:
			break;
		}
	}
}

