#ifndef Mem_add_h
#define Mem_add_h


#define FLAG_OPTIC_ADD    0
#define FLAG_PID_ADD      sizeof(char)
#define FLAG_RS_ADD       FLAG_PID_ADD+sizeof(char)

#define CALMAX_RS_ADD     20
#define CALMIN_RS_ADD     CALMAX_RS_ADD+sizeof(uint16_t)*8


#define KPID_ADD          CALMIN_RS_ADD+sizeof(uint16_t)*8
#define VBGS_ADD          KPID_ADD+sizeof(float)*3

void Write_EEPROM_size(int ADD,char* Data,int size);
void Read_EEPROM_size(int ADD,char* Data,int size);

#endif