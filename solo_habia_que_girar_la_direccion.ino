//Librerías necesarias


#include <SPI.h> //Comunicación SPI
#include <Wire.h>
#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>//Leer IMU
#include <LS7366.h> // Lectura Encoder


//Variables globales interrupción velocidad de avance
volatile unsigned long t_ant = 0;
volatile unsigned long t_new;
volatile unsigned long dt; // Tiempo para medir la velocidad de avance de la bicicleta

//Variables globales interrupción señales de radio
//---> Rueda
volatile unsigned long pulseInTimeBegin_inc; 
volatile unsigned long duration_inc;
//---> Gatillo
volatile unsigned long pulseInTimeBegin_vel;
volatile unsigned long duration_vel;

unsigned long t0_loop;


//Vaiables globales para la calibración IMU
float media_roll, media_yaw;
float media_sig0;
 
// Definición de los pines
//Telemetría
// Status LED
#define STATUS_LED LED_BUILTIN
// Output data settings
#define NFLOATS 7          // Number of floating point values per packet
#define NBYTES  4*NFLOATS   // Number of bytes per packet
#define dtloop 20 //(ms)
// Create file "network_data.h" with content:
//   #define  SECRET_SSID  "SSID"
//   #define  SECRET_PASS  "password"
#include "network_data.h"


static union {
  float out[NFLOATS];
  char replyBuffer[NBYTES];
};

// Delays for checking WiFi status and sending packets (ms)
const unsigned long WIFIDELAY = 5000;
const unsigned long SENDDELAY = 20;

unsigned long wifiTime = 0;
unsigned long sendTime = 0;

// WiFi parameters and status variables
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
unsigned connTrials = 0;
int wifiStatus = WL_IDLE_STATUS;
bool online = false;

// Network parameters
IPAddress remoteIP(192, 168, 137, 1);
unsigned int remotePort = 2390;
unsigned int localPort = 2390;
WiFiUDP UDPsocket;


#define PWM1 5
#define PWM2 6
#define CS 10
#define CS_PLACA 8
#define RUEDA A1  //D14 pone en PINOUT
#define GATILLO A0 //D15 pone en PINOUT
#define FOTVEL A4 //D18 pone en PINOUT
#define FOTDIR A5 //D19 pone en PINOUT
#define LED 7 // LED presente en el soporte trasero

LS7366 myLS7366(CS_PLACA); //Chip Select placa encoder. Hay que hacerlo global

//Ganancias del controlador y constantes presentes en el programa
#define k_phi -105.23140791 //-40.19  //-109.76879125   //-43.32 -40.19  //Ganancia ángulo de inclinación
#define k_delta 16.065156722 //10.96 //16.44990207 //10.96*0 //Ganancia ángulo de dirección
#define k_phip -17.7971412//-8.28 //-18.65979836  //-8.28 // Ganancia velocidad ángulo de inclinación
#define k_i -122.47448714 // Ganancia señal de error
#define K_I 50
#define k_p 100 // Ganancia proporcional
#define L 2*PI*0.254*1e6 // Longitud circunferencia rueda en micrómetros
#define g 9.81
#define h 2e6 //Tiempo mínimo en microsegundos que tiene que pasar entre dos cortes del testigo consecutivos para que la bicicleta se considere en movimiento
#define C 500 // Constante de amortiguamiento entrada de velocida
#define PWM_limit 200
#define roll_limit_deg 20





void setup() {

  pinMode(STATUS_LED, OUTPUT);    // LED de la placa auxiliar
  Wire.begin();
  pinMode (CS, OUTPUT);
  SPI.begin(); // Cuidado, se inicializa en la librería de lectura del encoder
  analogWrite(CS,0); //Ponemos el potenciómetro digital a cero para no acelerar en el arranque
  
  Serial.begin(115200);
  
   
   if (!IMU.begin())
   {
      Serial.println("Failed to initialize IMU!");
      while (1);
   }
   
  //Pines e interrupción velocidad de avance
  pinMode(FOTVEL, INPUT); // El pin analógico A0 se puede utilizar como digital --> D18
  attachInterrupt(digitalPinToInterrupt(FOTVEL),sensorRueda,FALLING); 

  //Pin señal de radio referencia roll (rueda)
  pinMode(RUEDA,INPUT);// El pin analógico A0 se puede utilizar como digital --> D14
  attachInterrupt(digitalPinToInterrupt(RUEDA),rueda,CHANGE);

  //Pin señal de radio referencia de velocidad (gatillo)
  pinMode(GATILLO,INPUT);// El pin analógico A1 se puede utilizar como digital --> D15
  attachInterrupt(digitalPinToInterrupt(GATILLO),gatillo,CHANGE);


  //Pin para recibir la señal del fotosensor del Homeing
  pinMode(FOTDIR,INPUT);

  //Pines PWM control del motor de la dirección
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);

  //Encendemos el LED trasero una vez tengamos alimentación en la placa
  pinMode(LED,OUTPUT);
  digitalWrite(LED,0);

 

  //Setup de la placa de lectura del encoder
  myLS7366.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4);
  myLS7366.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4 );
  myLS7366.clear_status_register();
  myLS7366.write_data_register(4);

  wifiStatus = WiFi.begin(ssid, pass);

  calibracionIMU();
  
  int estado = digitalRead(FOTDIR);
  homing(estado);
  
  
  digitalWrite(LED,1);
  t0_loop = micros();
  
  
}


void loop()
{

  static unsigned long t0_loop_tc;
  unsigned long t_loop_tc = millis();
  while(t_loop_tc-t0_loop_tc < dtloop)
  {
    t_loop_tc = millis();
  }
  t0_loop_tc +=  dtloop;
  //Control de la inclinación, comprobar que no sea muy brusco
  unsigned long tn_loop;
  float dt_loop;
  float ay = aceleracion_lateral();
  if (abs(ay) > 1)
  {
    analogWrite(CS,0);
    digitalWrite(PWM2,0);
    digitalWrite(PWM1,0);
    while(1)
    {
        digitalWrite(LED,1);
        delay(200);
        digitalWrite(LED,0); 
        delay(200);
    }
  }
    
  //Radiocontrol y fotosensor de velocidad
  static uint8_t sleep = 1;
  float v = L/dt;
  float ref;
  static float xi;
  if((micros()-t_new) > h)
  {
    v = 0;
    sleep = 0; 
  }
  else
  {
    sleep = 1; 
  }
  
  float ref_max = constrain(12*v-18.86,6,30);//Función que referencia el angulo de inclinación máximo en función de la velocidad
  
  if (duration_inc > media_sig0)
  {
    ref = mapfloat(duration_inc,media_sig0,1960,0,ref_max);
  }
  else
  {
     ref = mapfloat(duration_inc,1080,media_sig0,-ref_max,0);
  }

  ref = DegtoRad(ref);
  analogWrite(CS,(constrain(map(duration_vel,media_sig0,1065,0,205),0,205))); // Se manda señal [0,205] ---> Tensión de salida [0,4]V
  
  //Sensorización --> Medición de los 4 estados 
  static float w_pitch, w_roll, w_yaw;

  if(IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope( w_roll, w_pitch, w_yaw); //La velocidad de pitch no la queremos para nada
    w_roll = -DegtoRad(w_roll-media_roll);
    w_yaw = -DegtoRad(w_yaw-media_yaw);
  }
  
  float phi = kalman(w_roll, w_yaw, v);
  long deltapulses = -(myLS7366.read_counter()-180);
  float delta = deltapulses*2*PI/(64*150); //Ángulo de dirección. CUIDADO. si están conectados los cables del encoder al revés, hay que añadir un -
  float deltap = velocidad_direccion(delta);
  
  tn_loop = micros();
  dt_loop = (tn_loop-t0_loop)*1e-6 ;
  t0_loop = tn_loop;
  //Controlador inclinación---> A través de las ganacias obtenidas con el LQR
  xi = (xi + (ref-phi)*dt_loop)*sleep; // Señal de error  
  //xi = constrain(xi,-255/k_p/k_i,255/k_p/k_i);
  float u_v = -(k_phi*phi*sleep+k_delta*delta+k_phip*w_roll)+k_i*xi; //Entrada de velocidad
  
 //En este momento entramos en el control del motor
  double error = u_v-deltap;
  static double error_I = error*dt_loop;
  double PWM = k_p*error + K_I*error_I + C*w_roll*(1-sleep*1);
  

  if (PWM<0)
    {
    
    digitalWrite(PWM1,0);
    analogWrite (PWM2, constrain(-PWM, 0 , PWM_limit)); //GIRAR IZQUIERDA
    
    }
  else
    {
      
    digitalWrite (PWM2, 0);
    analogWrite (PWM1, constrain(PWM, 0 , PWM_limit)); //GIRAR DERECHA
  
    }


   //WiFi connection management
  if (wifiStatus != WL_CONNECTED) {
    online = false;

    if (t_loop_tc - wifiTime > WIFIDELAY) {
      //wifiStatus = WiFi.begin(ssid, pass);

      wifiTime += WIFIDELAY;
    }
  }
  else
  {
    if (!online) {
      UDPsocket.begin(localPort);
      online = true;
    }
  }

  if (online) {
    // WiFi check
    if (t_loop_tc - wifiTime > WIFIDELAY) {
      //wifiStatus = WiFi.status();
      wifiTime += WIFIDELAY;
    }

    // Send output data
    
    out[0] = t_loop_tc*1e-3;
    out[1] = phi;
    out[2] = w_roll;
    out[3] = ref;
    out[4] = deltap;
    out[5] = u_v ;
    out[6] = v;

    UDPsocket.beginPacket(remoteIP, remotePort);
    UDPsocket.write(replyBuffer, NBYTES);
    UDPsocket.endPacket();
    
  }
   
}
    

float DegtoRad(float entrada)
{
  
  float resultado = entrada*PI/180;
  
  return resultado;
}


void calibracionIMU()
{
  float suma_roll = 0, suma_yaw = 0;
  float w_pitch, w_roll, w_yaw;
  float suma_sign = 0;
  int cont;
  for(cont = 0; cont < 500; cont ++)
  {
     delay(10);  
     if (IMU.gyroscopeAvailable())
     {
        IMU.readGyroscope( w_roll,  w_pitch,  w_yaw);
     }
     suma_roll += w_roll;
     suma_yaw += w_yaw;
     suma_sign += duration_inc;
  }
  media_roll = suma_roll/cont; 
  media_yaw = suma_yaw/cont;
  media_sig0 = suma_sign/cont;
}

float aceleracion_lateral()
{
  float x,y,z;
  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(x, y, z);
  }
  return y;
}

void homing(int estado)
{

  while(estado == 1)
  {
     
     estado = digitalRead(FOTDIR);
     //METER PWM, hay que saber en que sentido gira el motor en función de que PIN se activa
     digitalWrite(PWM1,0);
     analogWrite (PWM2, 70);// GIRAR IZQUIERDA
     //long pos_corte = myLS7366.read_counter()*360/(64*150); //GRADOS
     
  }
  
  while(estado == 0)
  {
      
     estado = digitalRead(FOTDIR);
     //METER PWM, hay que saber en que sentido gira el motor en función de que PIN se activa
     digitalWrite(PWM2,0);
     analogWrite (PWM1, 70); // GIRAR DERECHA
     //long pos_corte = myLS7366.read_counter()*360/(64*150); //GRADOS
  }
  //Se ponen los dos pines PWM a 0, si no el motor seguiría girando
  digitalWrite(PWM2,0);
  digitalWrite (PWM1,0);
  myLS7366.clear_counter();
}

void sensorRueda() 
{
  t_new = micros();
  dt = (t_new - t_ant);
  t_ant = t_new;
}

void gatillo()
{
  if (digitalRead(GATILLO) == HIGH)
  {
    // start measuring
    pulseInTimeBegin_vel = micros();
  }
  else
  {
    // stop measuring
    duration_vel = micros()-pulseInTimeBegin_vel;
  }
}

void rueda()
{
  if (digitalRead(RUEDA) == HIGH)
  {
    // start measuring
    pulseInTimeBegin_inc = micros();
  }
  else
  {
    // stop measuring
    duration_inc = micros()-pulseInTimeBegin_inc;
  }
}


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float velocidad_direccion(float delta)
{
  //Serial.print(delta*180/PI);
  //Serial.print("\t");
  delta = delta*1e6; //Se escribe en microradianes para tener rad/s en el cálculo de deltap
  const int fc = 5;
  const float tau = 1/(2*PI*fc);
  static unsigned long t0;
  static float delta_0;
  unsigned long tn = micros();
  static float deltap_filt;
  float deltap = (delta-delta_0)/(tn-t0);
  //Serial.print(deltap*180/PI);
  //Serial.print("\t");
  float a = (tn-t0)*1e-6/(tau+(tn-t0)*1e-6);
  deltap_filt = (1-a)*deltap_filt+a*deltap; 

  delta_0 = delta;
  t0 = tn;
  return deltap_filt;

}

float kalman(float w_roll, float w_yaw, float v)
{
  
  float K = 0.029; //Ganancia de Kalman
  float  phi_s ;//Ángulo de inclinación a través de la integración del giróscopo
  float phi_v; //Ángulo de inclinación a través del sensor virtual
  static float phi; //Ángulo de inclinación combinando medidas de los dos sensores (estado de interés)
  static unsigned long t0= micros(); 
 
  unsigned long tn = micros(); //Tiempos para la integración
  float dt = (tn-t0)*1e-6;
  t0 = tn;
  phi_s = phi + w_roll*dt; //Integración del giróscopo (Predicción) Forward Euler como integrador:)
  phi_v = atan2(w_yaw*v,g); //Corrección con el sensor virtual (Innovación)
  phi = phi_s + K*(phi_v-phi_s); //Combinación de ambas medidas

  return phi;
}
