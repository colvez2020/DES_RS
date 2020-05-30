#ifndef Comumicacion_h
#define Comumicacion_h





#define MODO_AT     50
#define MODO_NORMAL 51



void Setup_Comunicacion(char MODO);

boolean Read_serial_bluethoot(char* DATA,char MODO);
void Write_serial_bluethoot(char DATA);
void Write_serial_bluethoot_stream(String DATA);
void Write_serial_bluethoot_stream_nl(String DATA);
void Write_serial_bluethoot_long(long DATA);
void Write_serial_bluethoot_uint16(uint16_t DATA);
void Write_serial_bluethoot_double(double DATA);
void Write_serial_bluethoot_float(float DATA);
void Write_serial_bluethoot_nl(void);
void Ejecutar_modoAT(void);

boolean Read_serial_Tableta(char* DATA);
void Parametros_consola_blue(int* base,float* Kprop,float* Kderiv,float* Kinte,int*orden);
#endif
