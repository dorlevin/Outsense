#include "mcu_api.h"
#include "mcu_errno.h"
#include <string.h>


void mcu_main()
{
	int len;
	char buf[64];
	while(1){
		 do {
			len = host_receive((unsigned char *)buf, 64);
			mcu_sleep(50);
		} while (len <= 0);
		if (strncmp(buf, "start", 5) == 0)
		{
			//debug_print(DBG_INFO, "registered a wake up call in 20 secs!\n");
			mcu_sleep(2000);
	        host_send((unsigned char*)"wake up!\n", 10);
		}
	}
}
