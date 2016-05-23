#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>   /* нужно для usbdrv.h */
#include "usbdrv/usbdrv.h"
#include "onewire.h"
#include "ds18x20.h"
#include "delay.h"


struct dataexchange_t       // Описание структуры для передачи данных
{
   uchar t1_1;
   uchar t1_2;
   uchar t1_3;
   uchar t2_1;
   uchar t2_2;
   uchar t2_3;
   uchar air;
   uchar version;
   uchar dev_id;
};

static struct dataexchange_t pdata =    {3, 0, 0, 3, 0, 0, 255, 1 ,0};
static struct dataexchange_t measures = {3, 0, 0, 3, 0, 0, 255, 1, 0};

/*----------------------------------------------------------------------*/
#define MAXSENSORS 2
uint8_t nSensors;
uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

volatile unsigned int int1_cnt = 0;
int air = 0;

void search_sensors(void)
{ 
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];	//OW_ROMCODE_SIZE = 8
	uint8_t diff;

	nSensors = 0;

	for( diff = OW_SEARCH_FIRST; diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ; )
    {
		DS18X20_find_sensor( &diff, &id[0] );
		for (i=0;i<OW_ROMCODE_SIZE;i++)
			gSensorIDs[nSensors][i]=id[i];
	
		nSensors++;
    }
 }

void measureTemp()
{
 	if (nSensors != 0)
	{
		if ( DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL ) == DS18X20_OK)
		{ 
			delay_ms(DS18B20_TCONV_12BIT);
			
			uchar subzero, cel, cel_frac_bits;
			if ( DS18X20_read_meas( &gSensorIDs[0][0], &subzero, &cel, &cel_frac_bits) == DS18X20_OK )
			{
				measures.t1_1 = subzero;
				measures.t1_2 = cel;
				measures.t1_3 = cel_frac_bits;
			}
			if (nSensors > 1)
			{
				if ( DS18X20_read_meas( &gSensorIDs[1][0], &subzero, &cel, &cel_frac_bits) == DS18X20_OK )
				{
					measures.t2_1 = subzero;
					measures.t2_2 = cel;
					measures.t2_3 = cel_frac_bits;
				}
			}
		}
	}
}
/*----------------------------------------------------------------------*/

const PROGMEM char usbHidReportDescriptor[22] = { // USB report descriptor         // Дескриптор описывает структуру пакета данных для обмена
    0x06, 0x00, 0xff,                       // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                             // USAGE (Vendor Usage 1)
    0xa1, 0x01,                             // COLLECTION (Application)
    0x15, 0x00,                             //    LOGICAL_MINIMUM (0)        // min. значение для данных
    0x26, 0xff, 0x00,                       //    LOGICAL_MAXIMUM (255)      // max. значение для данных, 255 тут не случайно, а чтобы уложиться в 1 байт
    0x75, 0x08,                             //    REPORT_SIZE (8)            // информация передается порциями, это размер одного "репорта" 8 бит
    0x95, sizeof(struct dataexchange_t),    //    REPORT_COUNT               // количество порций (у нашем примере = 3, описанная выше структура передастся за три репорта)
    0x09, 0x00,                             //    USAGE (Undefined)
    0xb2, 0x02, 0x01,                       //    FEATURE (Data,Var,Abs,Buf)
    0xc0                                    // END_COLLECTION
};

static uchar    currentAddress;
static uchar    bytesRemaining;
static uchar	needMeasure = 1;


/* usbFunctionRead() вызывается когда хост запрашивает порцию данных от устройства
 * Для дополнительной информации см. документацию в usbdrv.h
 */
uchar   usbFunctionRead(uchar *data, uchar len)
{
	PORTC ^= (1 << 5);
    if(len > bytesRemaining)
        len = bytesRemaining;

	pdata.t1_1 = measures.t1_1;
	pdata.t1_2 = measures.t1_2;
	pdata.t1_3 = measures.t1_3;
	pdata.t2_1 = measures.t2_1;
	pdata.t2_2 = measures.t2_2;
	pdata.t2_3 = measures.t2_3;
	pdata.air  = air;
	pdata.version = 1;
	pdata.dev_id = 137;
    uchar *buffer = (uchar*)&pdata;

	for (uchar j = 0; j < len; j++)
	{
		data[j] = buffer[j + currentAddress];
	}

    currentAddress += len;
    bytesRemaining -= len;

    return len;
}


/* usbFunctionWrite() вызывается когда хост отправляет порцию данных к устройству
 * Для дополнительной информации см. документацию в usbdrv.h
 */
uchar   usbFunctionWrite(uchar *data, uchar len)
{
    return 0; /* 0 означает, что есть еще данные */
}

ISR( INT1_vect )
{
  int1_cnt++;
}

/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR)
	{
        if(rq->bRequest == USBRQ_HID_GET_REPORT)
		{
			if (needMeasure) {
				measureTemp();
				GICR |= (1<<INT1);
				delay_ms(1000);
				GICR &= ~(1<<INT1);
				air = int1_cnt;
				int1_cnt = 0;
				needMeasure = 0;
			} else
				needMeasure = 1;
            bytesRemaining = sizeof(struct dataexchange_t);
            currentAddress = 0;
            return USB_NO_MSG;
		} 
		else if (rq->bRequest == USBRQ_HID_SET_REPORT)
		{
	        bytesRemaining = sizeof(struct dataexchange_t);
	        currentAddress = 0;
	        return USB_NO_MSG;
        }
    }
    return 0;
}
/* ------------------------------------------------------------------------- */

int main(void)
{
	
    usbInit();
    usbDeviceDisconnect();  // принудительно отключаемся от хоста, так делать можно только при выключенных прерываниях!
    
    _delay_ms(250);
    
    usbDeviceConnect();     // подключаемся

    sei();                  // разрешаем прерывания

    for(;;){                // главный цикл программы
        usbPoll();          // эту функцию надо регулярно вызывать с главного цикла, максимальная задержка между вызовами - 50 ms
	}
    return 0;
}
/* ------------------------------------------------------------------------- */ 

