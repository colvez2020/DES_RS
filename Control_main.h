#ifndef Control_main_h
#define Control_main_h

#define CALIBRA    60
#define NO_CALIBRA 61

void Setup_Seguidor_linea(char Modo);
void Mod_Parametros_PID(void);
void Ejecutar_seguidor_linea(void);
void Parametros_consola(void);
boolean Comando_valido(char a);
void Control_Bluethoot(char Comando);
void Contro_tableta(char comando);

#endif
