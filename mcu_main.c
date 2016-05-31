#include "mcu_api.h"
#include "mcu_errno.h"
#include <string.h>

//this function will run always on the MCU
void mcu_main()
{
	int len;
	char buf[64];
	while(1){
		//wait for wake up call to be registered by the host
		 do {
			len = host_receive((unsigned char *)buf, 64);
			mcu_sleep(50);
		} while (len <= 0);
		//if wake up call registered by host, then wait 20 secs, and then wake the host up
		if (strncmp(buf, "start", 5) == 0)
		{
			//debug_print(DBG_INFO, "registered a wake up call in 20 secs!\n");
			mcu_sleep(2000);
	        	host_send((unsigned char*)"wake up!\n", 10);
		}
	}
}
