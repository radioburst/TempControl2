#include "ds18b20.h"

void start_meas( void ){
  if( W1_IN & 1<< W1_PIN ){
    w1_command( CONVERT_T, NULL );
    W1_OUT |= 1<< W1_PIN;
    W1_DDR |= 1<< W1_PIN;			// parasite power on

  }
}


void read_meas( void )
{
	uchar id[9], diff;
	id[8] = 0;

	for( diff = SEARCH_FIRST; diff != LAST_DEVICE; )
	{
		diff = w1_rom_search( diff, id );

		if( diff == PRESENCE_ERR )
			break;

		if( diff == DATA_ERR )
			break;
		
		if( id[0] == 0x28 || id[0] == 0x10 )
		{
			// temperature sensor
			if(strcmp(id1, "00000000") == 0 && strcmp(id2, id) != 0)
			strcpy(id1, id);
			else if(strcmp(id2, "00000000") == 0 && strcmp(id1, id) != 0)
			strcpy(id2, id);

			w1_command( READ, id );
			//w1_byte_wr( READ );			// read command
			int16_t LSB = w1_byte_rd(); // low byte
			int16_t MSB = w1_byte_rd(); // high byte

			if(strcmp(id, id1) == 0)
			{
				uiMainSensorLastReading = 0; // Reset Error Value
				fTist = (float) (LSB + (MSB * 256)) / 16;
			}
			else if(strcmp(id, id2) == 0)
			{
				uiBeerSensorLastReading = 0; // Reset Error Value
				fTist2 = (float)  (LSB + (MSB * 256)) / 16;
			}
		}
	}
}
