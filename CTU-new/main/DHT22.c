/*------------------------------------------------------------------------------
	DHT22 temperature & humidity sensor AM2302 (DHT22) driver for ESP32
	Jun 2017:	Ricardo Timmermann, new for DHT22  	
	Code Based on Adafruit Industries and Sam Johnston and Coffe & Beer. Please help
	to improve this code. 
---------------------------------------------------------------------------------*/

#include "include/DHT22.h"

// == global defines =============================================

static const char* TAG = "DHT";

int DHTgpio = DHT_GPIO;				
float humidity = 0.;
float temperature = 0.;

struct tm temp_time;
const char* temp_path = "warwicktrial/ambient/temperature";
const char* hum_path = "warwicktrial/ambient/humidity";

extern const char debug[80];

// == set the DHT used pin=========================================

void setDHTgpio( int gpio )
{
	DHTgpio = gpio;
}

// == get temp & hum =============================================

float getHumidity() { return humidity; }
float getTemperature() { return temperature; }

// == error handler ===============================================

void errorHandler(int response)
{
	switch(response) {
	
		case DHT_TIMEOUT_ERROR :
			ESP_LOGE( TAG, "Sensor Timeout\n" );
			break;

		case DHT_CHECKSUM_ERROR:
			ESP_LOGE( TAG, "CheckSum error\n" );
			break;

		case DHT_OK:
			break;

		default :
			ESP_LOGE( TAG, "Unknown error\n" );
	}
}

/*-------------------------------------------------------------------------------
;
;	get next state 
;
;	I don't like this logic. It needs some interrupt blocking / priority
;	to ensure it runs in realtime.
;
;--------------------------------------------------------------------------------*/

int getSignalLevel( int usTimeOut, bool state )
{

	int uSec = 0;
	while( gpio_get_level(DHTgpio)==state ) {

		if( uSec > usTimeOut ) 
			return -1;
		
		++uSec;
		ets_delay_us(1);		// uSec delay
	}
	
	return uSec;
}

/*----------------------------------------------------------------------------
;
;	read DHT22 sensor
copy/paste from AM2302/DHT22 Docu:
DATA: Hum = 16 bits, Temp = 16 Bits, check-sum = 8 Bits
Example: MCU has received 40 bits data from AM2302 as
0000 0010 1000 1100 0000 0001 0101 1111 1110 1110
16 bits RH data + 16 bits T data + check sum
1) we convert 16 bits RH data from binary system to decimal system, 0000 0010 1000 1100 → 652
Binary system Decimal system: RH=652/10=65.2%RH
2) we convert 16 bits T data from binary system to decimal system, 0000 0001 0101 1111 → 351
Binary system Decimal system: T=351/10=35.1°C
When highest bit of temperature is 1, it means the temperature is below 0 degree Celsius. 
Example: 1000 0000 0110 0101, T= minus 10.1°C: 16 bits T data
3) Check Sum=0000 0010+1000 1100+0000 0001+0101 1111=1110 1110 Check-sum=the last 8 bits of Sum=11101110
Signal & Timings:
The interval of whole process must be beyond 2 seconds.
To request data from DHT:
1) Sent low pulse for > 1~10 ms (MILI SEC)
2) Sent high pulse for > 20~40 us (Micros).
3) When DHT detects the start signal, it will pull low the bus 80us as response signal, 
   then the DHT pulls up 80us for preparation to send data.
4) When DHT is sending data to MCU, every bit's transmission begin with low-voltage-level that last 50us, 
   the following high-voltage-level signal's length decide the bit is "1" or "0".
	0: 26~28 us
	1: 70 us
;----------------------------------------------------------------------------*/

#define MAXdhtData 5	// to complete 40 = 5*8 Bits

int readDHT()
{
int uSec = 0;

uint8_t dhtData[MAXdhtData];
uint8_t byteInx = 0;
uint8_t bitInx = 7;

	for (int k = 0; k<MAXdhtData; k++) 
		dhtData[k] = 0;

	// == Send start signal to DHT sensor ===========

	gpio_set_direction( DHTgpio, GPIO_MODE_OUTPUT );

	// pull down for 3 ms for a smooth and nice wake up 
	gpio_set_level( DHTgpio, 0 );
	ets_delay_us( 3000 );			

	// pull up for 25 us for a gentile asking for data
	gpio_set_level( DHTgpio, 1 );
	ets_delay_us( 25 );

	gpio_set_direction( DHTgpio, GPIO_MODE_INPUT );		// change to input mode
  
	// == DHT will keep the line low for 80 us and then high for 80us ====

	uSec = getSignalLevel( 85, 0 );
//	ESP_LOGI( TAG, "Response = %d", uSec );
	if( uSec<0 ) return DHT_TIMEOUT_ERROR; 

	// -- 80us up ------------------------

	uSec = getSignalLevel( 85, 1 );
//	ESP_LOGI( TAG, "Response = %d", uSec );
	if( uSec<0 ) return DHT_TIMEOUT_ERROR;

	// == No errors, read the 40 data bits ================
  
	for( int k = 0; k < 40; k++ ) {

		// -- starts new data transmission with >50us low signal

		uSec = getSignalLevel( 56, 0 );
		if( uSec<0 ) return DHT_TIMEOUT_ERROR;

		// -- check to see if after >70us rx data is a 0 or a 1

		uSec = getSignalLevel( 75, 1 );
		if( uSec<0 ) return DHT_TIMEOUT_ERROR;

		// add the current read to the output data
		// since all dhtData array where set to 0 at the start, 
		// only look for "1" (>28us us)
	
		if (uSec > 40) {
			dhtData[ byteInx ] |= (1 << bitInx);
			}
	
		// index to next byte

		if (bitInx == 0) { bitInx = 7; ++byteInx; }
		else bitInx--;
	}

	// == get humidity from Data[0] and Data[1] ==========================

	humidity = dhtData[0];
	humidity *= 0x100;					// >> 8
	humidity += dhtData[1];
	humidity /= 10;						// get the decimal

	// == get temp from Data[2] and Data[3]
	
	temperature = dhtData[2] & 0x7F;	
	temperature *= 0x100;				// >> 8
	temperature += dhtData[3];
	temperature /= 10;

	if( dhtData[2] & 0x80 ) 			// negative temp, brrr it's freezing
		temperature *= -1;


	// == verify if checksum is ok ===========================================
	// Checksum is the sum of Data 8 bits masked out 0xFF
	
	if (dhtData[4] == ((dhtData[0] + dhtData[1] + dhtData[2] + dhtData[3]) & 0xFF)) 
		return DHT_OK;

	else 
		return DHT_CHECKSUM_ERROR;
}

void CTU_ambient_temperature(void *arg)
{
    int ret = readDHT();

    errorHandler(ret);

    if (!ret)
    {
        //ESP_LOGI(TAG, "Humidity %.1f", getHumidity());
        //ESP_LOGI(TAG, "Temperature %.1f", getTemperature());
		char temp[20], hum[20];
		time(&now);
        localtime_r(&now, &temp_time);

		if (SD_CARD)
		{
			write_sd_card(temp_path, getTemperature(), &temp_time);
			write_sd_card(hum_path, getHumidity(), &temp_time);
		}
		if (MQTT)
		{
        	sprintf(temp, "%.1f", getTemperature());
        	esp_mqtt_client_publish(client, temp_path, temp, 0, 0, 0);
			sprintf(hum, "%.1f", getHumidity());
        	esp_mqtt_client_publish(client, hum_path, hum, 0, 0, 0);
		}

		TOO_COLD = false;

		if ( ( getTemperature() < MIN_TEMP_LIMIT ) && ( m_CTU_task_param.state == CTU_POWER_TRANSFER_STATE ) )
		{
			/*
			//switch all pads off
			struct peer *peer;
			SLIST_FOREACH(peer, &peers, next) {
				if (!peer->CRU) 
				{
					if(full_power_pads[peer->position-1])
						ble_central_update_control_enables(0, 1, 0, peer);
					else if(low_power_pads[peer->position-1])
							ble_central_update_control_enables(0, 0, 0, peer); 
				}
			}

			TOO_COLD = true;
			if (m_CTU_task_param.state != CTU_CONFIG_STATE )
				CTU_state_change( CTU_CONFIG_STATE, NULL );
			*/
			if (MQTT)
				esp_mqtt_client_publish(client, debug, "TOO COLD", 0, 0, 0);
		}

    }

	time(&reconn_time);

    //NVS reading
    esp_err_t err = nvs_open("reconnection", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else 
    {
        // Read 
        nvs_get_i64(my_handle, "pad1", &timePad[0]);
        nvs_get_i64(my_handle, "pad2", &timePad[1]);
        nvs_get_i64(my_handle, "pad3", &timePad[2]);
        nvs_get_i64(my_handle, "pad4", &timePad[3]);
        nvs_get_i64(my_handle, "3PAU", &timeScooter[VOI_3PAU]);
        nvs_get_i64(my_handle, "6F35", &timeScooter[VOI_6F35]);
        nvs_get_i64(my_handle, "CE8J", &timeScooter[VOI_CE8J]);
        nvs_get_i64(my_handle, "D8X5", &timeScooter[VOI_D8X5]);
        
        // Close
        nvs_close(my_handle);
    }
}
