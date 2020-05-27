#include <EEPROM.h>
 # include "Arduino.h"
 # include "OpenLamborghino.h"
 # include "Mem_add.h"
#include "Comunicacion.h"


 # define NUM_SENSORS 7 // number of sensors used
 # define NUM_SAMPLES_PER_SENSOR 4 // average 4 analog samples per sensor reading
 # define EMITTER_PIN 20 // emitter is controlled by digital


 # define Drueda 25
 //# define HIZ A7 // pin Hito Izquierda
 //# define HDE A6 // pin Hito Derecha


// int umbral = 650;
// int fin = 0;

//int BOTON;
int BUZZER;

// long ppr = 60.0;

// long PasosIz = 0;
// long PasosDe = 0;

// long lPasosIz = 0;
// long lPasosDe = 0;

// long n = 0;
// long m = 0;

// long piz = 0;
// long pde = 0;

// long timeNow = 0;
// long timeBefore = 0;
// long sampletime = 15000; //microsegundos

// long StimeNow = 0;
// long StimeBefore = 0;
// long Ssampletime = 100000; //microsegundos

// long FBtimeNow = 0;
// long FBtimeBefore = 0;
// long FBsampletime = 15000; //microsegundos


// float velrealiz = 0; //  velrealiz = (d2-d1)/(timeNow-timeBefore)
// float velrealde = 0; //  velrealiz = (d2-d1)/(timeNow-timeBefore)
// float velprom = 0;

// int setv = 100;
// int PowIz = 0;
// int PowDe = 0;
// int Pow = 0;

//--------------------PIDLambo--------------------------------------------
double position;
int derivative = 0; // derivada
int proportional = 0; // proporcional
int integral =0;
int power_difference = 0; // velocidad diferencial
int last_proportional;
//int RANGEBRAKE = 2500;

// int error1 = 0;
// int error2 = 0;
// int error3 = 0;
// int error4 = 0;
// int error5 = 0;
// int error6 = 0;


//Control PID

float KP;
float KD;
float KI;

// //--------------------Hitos--------------------------

// int contHito = 0;
// int geo = 0;
// int geo1 = 0;
// int geo2 = 0;
// int geo3 = 0;
// int geo4 = 0;
// int geo5 = 0;

// int Hiz = 0;
// int Hde = 0;

// //---------------mapa----------------

// int x = 0;

// int program = 1;

// //Variables directas (obtenidas)
// int mapade[50]; //pasos encoder
// int mapaiz[50]; //pasos encoder
// int mapa[50];
// int rmapa[50];
// int velocidades[50];
// int mapcontador = 0;

// //----------------- Velocidades

// int vrecta = 1000;
// int vcurva = 500;

QTRSensors qtra;

uint16_t sensorValues[NUM_SENSORS];

OpenLamborghino::OpenLamborghino(int PINBUZZER) {
	BUZZER = PINBUZZER;
	//BOTON = PINBOTON;
	//pinMode(PINBOTON, INPUT);
//	pinMode(HIZ, INPUT);
//	pinMode(HDE, INPUT);
	pinMode(PINBUZZER, OUTPUT);
	qtra.setTypeAnalog();
    qtra.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6}, NUM_SENSORS);
    qtra.setEmitterPin(EMITTER_PIN);
    qtra.setSamplesPerSensor(NUM_SAMPLES_PER_SENSOR);


}

// void OpenLamborghino::WaitBoton() {   // Entra en un bucle infinito de espera.
// 	while (!digitalRead(BOTON));  // Se sale del bucle cuando se aprieta el botón
// 	tone(BUZZER, 2000, 100);      // Cuando sale del bucle, suena el buzzer
// }

void OpenLamborghino::beep() {
	tone(BUZZER, 2000, 100);  // Hce sonar el buzzer de Open Lamborghino por 100ms a 2000 
}


// void OpenLamborghino::IfBoton() {
// 	if (digitalRead(BOTON) == HIGH) 	{
// 		tone(BUZZER, 1000, 50);
// 		delay(200);
// 		WaitBoton();
// 		delay(200);
// 	}
// }



void OpenLamborghino::calibracion() {

	tone(BUZZER, 1000, 100);
	for (int i = 0; i < 400; i++) // make the calibration take about 10 seconds
	{
		qtra.calibrate(); // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
	}

//	Serial.begin(9600);
	for (int i = 0; i < NUM_SENSORS; i++)
	 {
		EEPROM.put(CALMIN_RS_ADD+sizeof(uint16_t)*i, qtra.calibrationOn.minimum[i]);
		Write_serial_bluethoot_uint16(qtra.calibrationOn.minimum[i]);
		Write_serial_bluethoot(' ');
	}
	Write_serial_bluethoot_nl();

	for (int i = 0; i < NUM_SENSORS; i++) 
	{
		EEPROM.put(CALMAX_RS_ADD+sizeof(uint16_t)*i, qtra.calibrationOn.maximum[i]);
		Write_serial_bluethoot_uint16(qtra.calibrationOn.maximum[i]);
		Write_serial_bluethoot(' ');
	}
	Write_serial_bluethoot_nl();
	Write_serial_bluethoot_nl();
	tone(BUZZER, 1500, 50);
	delay(70);
	tone(BUZZER, 1500, 50);
}


void OpenLamborghino::Getcalibracion() 
{

	tone(BUZZER, 1000, 100);
	for (int i = 0; i < 1; i++) // make the calibration take about 10 seconds
	{
		qtra.calibrate(); // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
	}

	Serial.begin(9600);
	for (int i = 0; i < NUM_SENSORS; i++) {
		EEPROM.get(CALMIN_RS_ADD+sizeof(uint16_t)*i,qtra.calibrationOn.minimum[i]);
		//Read_EEPROM_size(     CALMIN_RS_ADD+sizeof(uint16_t)*i,
		//				 (char*)&qtra.calibrationOn.minimum[i],
		 //				 					  sizeof(uint16_t));
		Write_serial_bluethoot_uint16(qtra.calibrationOn.minimum[i]);
		Write_serial_bluethoot(' ');
	}
	Write_serial_bluethoot_nl();

	for (int i = 0; i < NUM_SENSORS; i++) {
		EEPROM.get(CALMAX_RS_ADD+sizeof(uint16_t)*i,qtra.calibrationOn.maximum[i]);
		//Read_EEPROM_size(     CALMAX_RS_ADD+sizeof(uint16_t)*i,
		//				 (char*)&qtra.calibrationOn.maximum[i],
		 //				 					  sizeof(uint16_t));

		Write_serial_bluethoot_uint16(qtra.calibrationOn.maximum[i]);
		Write_serial_bluethoot(' ');
	}
	Write_serial_bluethoot_nl();
	Write_serial_bluethoot_nl();

	EEPROM.get(KPID_ADD,KP);
	EEPROM.get(KPID_ADD+sizeof(float)*1,KI);
	EEPROM.get(KPID_ADD+sizeof(float)*2,KD);
	
	tone(BUZZER, 1500, 50);
	delay(70);
	tone(BUZZER, 1500, 50);
}

long OpenLamborghino::LineaNegra() {
	long respuesta;
	double Sensores_Normalizado[NUM_SENSORS];
	double Polinomio=0;
	uint16_t posicion = qtra.readLineBlack(sensorValues);

	for (uint8_t i = 0; i < NUM_SENSORS; i++)
  	{
    	//Write_serial_bluethoot_uint16(sensorValues[i]);
    	//Write_serial_bluethoot_stream(" ");
    	if(sensorValues[i]>850) // Hay cienta negra
    	{  
    		Sensores_Normalizado[i]=sensorValues[i]/1000.0;
    	}
    	else
    	{
    		Sensores_Normalizado[i]=0;
    	}
    }

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
  	{

		Write_serial_bluethoot_double(Sensores_Normalizado[i]);
		Write_serial_bluethoot_stream("   ");
		if(Sensores_Normalizado[i]!=0)
			Polinomio=(i+1)*1000*Sensores_Normalizado[i];
		if(Sensores_Normalizado[i+1]!=0)
		{
			Polinomio=(Polinomio+Sensores_Normalizado[i+1]*(i+2)*1000)/2;
		}	
		// if(i!=0 &&)
		// {

		// 	Polinomio=(Sensores_Normalizado[i-1]*((i)*1000)+Sensores_Normalizado[i]*((i+1)*1000))/2;	
		// }
		// else
		// {
		// 	Polinomio=Sensores_Normalizado[i]*((i+1)*1000);
		// }
  	}
 	Write_serial_bluethoot_stream("POS: ");
   	Write_serial_bluethoot_double(Polinomio);
 //  	Write_serial_bluethoot_stream(" ");
	// respuesta = map(posicion, 1900, 3700, -255, 255);
	// Write_serial_bluethoot_stream("RP:");
	// Write_serial_bluethoot_long(respuesta);

	Write_serial_bluethoot_nl();

	//1   2   3   4  5  6  7

	return respuesta;
}

long OpenLamborghino::LineaBlanca() {
	uint16_t posicion = qtra.readLineWhite(sensorValues);
	posicion = map(posicion, 0, 5000, -255, 255);
	return posicion;
}

void OpenLamborghino::PIDLambo(float kp, float kd, float ki) {

	KP = kp;
	KI = ki;
	KD = kd;
	EEPROM.put(KPID_ADD,KP);
	EEPROM.put(KPID_ADD+sizeof(float)*1,KI);
	EEPROM.put(KPID_ADD+sizeof(float)*2,KD);
}

//setpoint=0; 
long OpenLamborghino::PID(int POS, int setpoint, int lim) {

	//Proporcional 
	proportional = int(POS) - setpoint;
	//Derivativo
	derivative = proportional - last_proportional;
	last_proportional = proportional;
	//Integal
	integral=integral+int(POS) - setpoint;

	//Accion de contorl //Lim = sobre inmpulso maximo. 
	int power_difference = (proportional * KP) + (derivative * KD) + (integral*KI);
	if (power_difference > lim)
		power_difference = lim;
	else if (power_difference < -lim)
		power_difference = -lim;
	
	return power_difference;
}




// void OpenLamborghino::EncoderIz() {
// 	PasosIz++;
// 	piz++;
// }

// void OpenLamborghino::EncoderDe() {
// 	PasosDe++;
// 	pde++;
// }

// void OpenLamborghino::vel() {

// 	timeNow = micros();
// 	if ((timeNow - timeBefore) > (sampletime)) {
// 		velrealiz = ((Drueda * 3.14) / ppr) * (1000000.0 / sampletime) * (PasosIz - n);
// 		velrealde = ((Drueda * 3.14) / ppr) * (1000000.0 / sampletime) * (PasosDe - m);
// 		velprom = (velrealiz + velrealde) / 2;
// 		timeBefore = timeNow;
// 		controlador();
// 		n = PasosIz;
// 		m = PasosDe;
// 	}
// }

// void OpenLamborghino::incomingByte(int a) {
// 	velocid = a;
// }

// int OpenLamborghino::controlador() {
// 	FBtimeNow = micros();
// 	if ((FBtimeNow - FBtimeBefore) > (FBsampletime)) {
// 		if (velocid == 0) {
// 			Pow = 0;
// 		}
// 		if (velprom < velocid) {
// 			Pow++;
// 			if (pow > 255) {
// 				pow = 255;
// 			}
// 		} else {
// 			Pow--;
// 			if (pow < 0) {
// 				pow = 0;
// 			}
// 		}
// 		FBtimeBefore = FBtimeNow;
// 	}
// 	return Pow;
// }






// void OpenLamborghino::comunicacion() {
// 	StimeNow = micros();
// 	if ((StimeNow - StimeBefore) > (Ssampletime)) {
// 		Serial.println(velprom);
// 		StimeBefore = StimeNow;
// 	}
// }

// void OpenLamborghino::ReadSensors() {
// 	Hiz = analogRead(HIZ);
// 	Hde = analogRead(HDE);

// 	if (Hiz < umbral) {
// 		Hiz = 1;
// 	} else {
// 		Hiz = 0;
// 	}

// 	if (Hde < umbral) {
// 		Hde = 1;
// 	} else {
// 		Hde = 0;
// 	}
// }

// void OpenLamborghino::detecGeo() {

// 	ReadSensors();

// 	if (Hiz == 0 && Hde == 0) {
// 		geo = 0;
// 	}
// 	if (Hiz == 1 && Hde == 0) {
// 		geo = 1;
// 	}
// 	if (Hiz == 0 && Hde == 1) {
// 		geo = 2;
// 	}
// 	if (Hiz == 1 && Hde == 1) {
// 		geo = 3;
// 	}

// 	if (geo1 != geo) {
// 		if (geo == 0 && geo1 == 1 && geo2 == 0) {
// 			fin++;

// 			funcionHitoDe();

// 		}

// 		if (geo == 0 && geo1 == 2 && geo2 == 0) {

// 			funcionHitoIz();
// 		}

// 		if (geo == 0 && ((geo1 == 3) || (geo2 == 3) || (geo3 == 3))) {

// 			funcionCruce();


// 		}
// 		geo5 = geo4;
// 		geo4 = geo3;
// 		geo3 = geo2;
// 		geo2 = geo1;
// 		geo1 = geo;
// 	}
// }

// int OpenLamborghino::change() {
// 	int changer = program;
// 	return changer;
// }

// void OpenLamborghino::velocimetro() {
// 	velocid = velocidades[mapcontador];
// }


// void OpenLamborghino::funcionCruce() {
// 	Serial.println("Intersection");
// }


// void OpenLamborghino::funcionHitoIz() {
// tone(BUZZER, 1500, 50);

// 			mapaiz[mapcontador] = int(0.335 * (piz));
// 			mapade[mapcontador] = int(0.335 * (pde));
// 			mapa[mapcontador] = int(0.5 * (mapaiz[mapcontador] + mapade[mapcontador]));
// 			rmapa[mapcontador] = abs(int(6 * (mapaiz[mapcontador] + mapade[mapcontador]) / ((mapaiz[mapcontador] - mapade[mapcontador]))));

// 			piz = 0;
// 			pde = 0;

			
// 			// Serial.print(velocid);
// 			// Serial.print("\t");
// 			// Serial.print(mapaiz[mapcontador]);
// 			// Serial.print("\t");
// 			// Serial.print(mapade[mapcontador]);
// 			// Serial.print("\t");
// 			// Serial.println("Left Marker");
			 
// 			mapcontador++;
// }


// void OpenLamborghino::funcionHitoDe() {

// 			tone(BUZZER, 2000, 50);

// 			mapaiz[mapcontador] = int(0.335 * (piz));
// 			mapade[mapcontador] = int(0.335 * (pde));
// 			mapa[mapcontador] = int(0.5 * (mapaiz[mapcontador] + mapade[mapcontador]));
// 			rmapa[mapcontador] = abs(int(6 * (mapaiz[mapcontador] + mapade[mapcontador]) / ((mapaiz[mapcontador] - mapade[mapcontador]))));

// 			piz = 0;
// 			pde = 0;
			
			
// 			// Serial.print(velocid);
// 			// Serial.print("\t");
// 			// Serial.print(mapaiz[mapcontador]);
// 			// Serial.print("\t");
// 			// Serial.print(mapade[mapcontador]);
// 			// Serial.print("\t");
// 			// Serial.println("Right Marker");
			 
			 
// 			mapcontador++;

			
// 			if (fin >= 2) {
// 				velocid = 0;
// 				program++;
// 				Serial.println(program);
// 				mapcontador = 0;
// 				fin = 0;
// 			}
			
			
// }

// void OpenLamborghino::datalogger() {

// 	Serial.print("Mapiz:   ");
// 	for (int i = 0; i <= 255; i++) {
// 		if (mapaiz[i] == 0) {
// 			break;
// 		}
// 		Serial.print(mapaiz[i]);
// 		Serial.print("\t");
// 	}

// 	Serial.println();
// 	Serial.print("Mapde:   ");

// 	for (int i = 0; i <= 255; i++) {
// 		if (mapaiz[i] == 0) {
// 			break;
// 		}
// 		Serial.print(mapade[i]);
// 		Serial.print("\t");
// 	}

// 	Serial.println();
// 	Serial.print("Mapa:   ");

// 	for (int i = 0; i <= 255; i++) {
// 		if (mapaiz[i] == 0) {
// 			break;
// 		}
// 		Serial.print(mapa[i]);
// 		Serial.print("\t");
// 	}

// 	Serial.println();
// 	Serial.print("Rmapa:   ");

// 	for (int i = 0; i <= 255; i++) {
// 		if (mapaiz[i] == 0) {
// 			break;
// 		}
// 		Serial.print(rmapa[i]);
// 		Serial.print("\t");
// 	}
// 	Serial.println("\t");
// }
