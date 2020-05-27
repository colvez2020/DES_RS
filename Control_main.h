#ifndef Control_main_h
#define Control_main_h

//Segidor linea


#define FLAG_OPTIC_OK     30
#define FLAG_PID_OK       31
#define FLAG_PID_RESET    32
#define FLAG_RS_OK        33
#define FLAG_RS_RESET     34


#define CALIBRA           60
#define NO_CALIBRA        61
#define BUZZER_PIN        10

void Setup_Seguidor_linea(uint8_t modo);
boolean Sintonizar_PID(void);
void Getparametros_VB(int* VB );
void Setparametros_VB(int VB );
void Ejecutar_seguidor_linea(void);
boolean Comando_valido(char a);
void Control_Bluethoot(char Comando);
void Contro_tableta(char comando);
long Leer_sensor(void);

#endif
