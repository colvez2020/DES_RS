#include <EEPROM.h>
#include "Mem_add.h"

void Write_EEPROM_size(int ADD,char* Data,int size)
{
	for(int i=0;i<size;i++)
	{
		EEPROM.write(ADD+i,Data[i]);
	}

}

void Read_EEPROM_size(int ADD,char* Data,int size)
{
	for(int i=0;i<size;i++)
	{
		Data[i]=EEPROM.read(ADD+i);
	}

}