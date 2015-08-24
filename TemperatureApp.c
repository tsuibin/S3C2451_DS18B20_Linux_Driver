#include <stdio.h>

int main()
{
	FILE *fp0 = NULL;
	char Buf[4096];
	
	/*初始化Buf*/
	
	/*打开设备文件*/
	fp0 = fopen("/dev/ds18b20","r+");
	if (fp0 == NULL)
	{
		printf("Open Temperature Device Error!\n");
		return -1;
	}

	while(1)
	{
		usleep(500* 1000);
		fread(Buf, sizeof(Buf), 2, fp0);
		printf("\n Temperature =%d.%02d\n",Buf[0],Buf[1]);
	}
	
	return 0;	

}
