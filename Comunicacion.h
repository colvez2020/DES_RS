#ifndef Comumicacion_h
#define Comumicacion_h

#define MODO_AT     50
#define MODO_NORMAL 51

void Setup_Comunicacion(char MODO);
boolean Read_serial_bluethoot(char* DATA);
void Write_serial_bluethoot(char DATA);
boolean Read_serial_Tableta(char* DATA);
#endif
