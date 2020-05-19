#ifndef Sistema_Usonic_h
#define Sistema_Usonic_h

#define USONIC_TEST   80
#define USONIC_NORMAL 81

void Setup_Usonic(void);
float Read_Usonic(char MODO);
float doMeasurement(void);

#endif
