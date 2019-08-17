/******** INCLUDE ********/


#include <PinChangeInt.h> // https://www.teachmemicro.com/arduino-interrupt-tutorial/
//cette library fonctionne pas si un des timers est déjà défini ici
#include <Wire.h> 
#include <digitalWriteFast.h> 
#include <VarSpeedServo.h> // https://github.com/netlabtoolkit/VarSpeedServo V. 11 Apri 2018
// Utilise aussi le timer2, comme la library Servo.h


/******** VARIABLES ********/


/**** Constantes ****/
#define pi 3.14159265
#define STEPS_PER_MM 87.49
#define res_mot 3200 //3200 steps par tour
#define res_enc 8192 //résolution de 8192 pour les encodeurs
#define RAD_PER_STEP (2*pi/res_mot) 
#define MM_PER_RAD (1/(RAD_PER_STEP*STEPS_PER_MM))
#define RAD_PER_MM (RAD_PER_STEP*STEPS_PER_MM)


/**** Timer et frequences ****/
int f_Controller; 
float T_Controller;
int unsigned compt_Controller;
int unsigned compt_controle = 0;
int unsigned compt_MoteurX;
int unsigned compt_MoteurY;
int unsigned compt_time=0;
float f_feedforward_X = 0.0;
float f_feedforward_Y = 0.0;
int unsigned compt = 0;
int unsigned compt_pulseMoteurX=0;
int unsigned compt_pulseMoteurY=0;
int unsigned compt_ext=0;
float T_e; //= (float)1/500; //freq de 500Hz
float f_e; //ICI changer freq en fonction du timer choisi !! 
float freq_max;
float f_ext;
float T_ext;

/**** Moteurs ****/ 
//Moteur selon X
#define STEP_PIN_X 9
#define DIR_PIN_X 3
//Moteur selon Y
#define STEP_PIN_Y 10
#define DIR_PIN_Y 11

/**** Encoder ****/
#define Encoder_PINA_X  13
#define Encoder_PINB_X  12
volatile long Pos_mes_X = 0;
#define Encoder_PINA_Y  A3
#define Encoder_PINB_Y  A2
volatile long Pos_mes_Y = 0;

/**** Position mesurée ****/
float pos_mes_Y_rad; //en radians
float pos_mes_X_rad; //en radians

/**** Reference ****/
float pos_ref_X;  //en radians //référence à l'instant t
float pos_ref_Y;  //en radians
float vitesse_ref_X; // pente de la consigne de rampe
float pos_ref_final_X; // Fin de la consigne de rampe
float vitesse_ref_Y;  
float pos_ref_final_Y; 
float pos_ref_depart_X = 0.0; //Début de la consigne de rampe
float pos_ref_depart_Y = 0.0; //Début de la consigne de rampe

/**** Limit Pins ****/
#define LIMIT_PIN_X 8
#define LIMIT_PIN_Y 2

/**** Contrôleur Proportionnel   ****/
float Kp_X;
float Kp_Y;
float erreur_X;
float erreur_Y;

/**** Servo axe Z ****/
bool Servo_activated = 0; //Pour permettre mvt servo
int Servo_High = 120; //position basse 
int Servo_Low = 150; //position haute
int Servo_SlowSpeed = 10; //speed from 1(slow) to 255(fast)
VarSpeedServo myservo; //create servo object to control a servo with variable speed
#define SERVO_PIN A1 //sur port 7

/**** Variables de commandes ****/
float Command_u = 0.0; //ICI INUTILE
float Command_v = 0.0;
float command_freq_Y;
float command_freq_X;
int DIR_X_Command = 0;
int DIR_Y_Command = 0;

/**** Pour lecture et envoi ****/
String ReadingSerial; //Lecture commande envoyée par PC
String ToSend = ""; //Regroupe les positions à envoyer au PC
bool Comptage_desactive = 0; //Désactivation incrémentation des compteurs
const int NUMBER_OF_FIELDS = 2; // how many comma separated fields we expect
float values[NUMBER_OF_FIELDS];   // array holding values for all the fields
int fieldIndex = 0;            // the current field being received
int NbreCharacters = 15; //Nbre de caractères communication
char a = 'b'; //character for handshake


/******** INTERRUPTIONS ********/


/****   Timer 2   ****/
ISR(TIMER2_COMPA_vect){
   //interrupt commands for TIMER 2 here
   if(Comptage_desactive == 0)
   {
      //Incrémentation des compteurs d'interrupt
      compt_ext+=1;
      compt_controle+=1;
      compt_pulseMoteurX+=1;
      compt_pulseMoteurY+=1;
   }
}

/****   Encoders   ****/
/** Encodeur axe X **/
//interrupt pinA encoder
void doEncoderA_X() {
  // look for a low-to-high on channel A
  if (digitalRead(Encoder_PINA_X) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(Encoder_PINB_X) == LOW) {
      Pos_mes_X = Pos_mes_X - 1;         // CW
    }
    else {
      Pos_mes_X = Pos_mes_X + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(Encoder_PINB_X) == HIGH) {
      Pos_mes_X = Pos_mes_X - 1;          // CW
    }
    else {
      Pos_mes_X = Pos_mes_X + 1;          // CCW
    }
  }
}
//interrupt pinB encoder
void doEncoderB_X() {
  // look for a low-to-high on channel B
  if (digitalRead(Encoder_PINB_X) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(Encoder_PINA_X) == HIGH) {
      Pos_mes_X = Pos_mes_X - 1;         // CW
    }
    else {
      Pos_mes_X = Pos_mes_X + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(Encoder_PINA_X) == LOW) {
      Pos_mes_X = Pos_mes_X - 1;          // CW
    }
    else {
      Pos_mes_X = Pos_mes_X + 1;          // CCW
    }
  }
}
/** Encodeur axe Y **/
//interrupt pinA encoder
void doEncoderA_Y() {
  // look for a low-to-high on channel A
  if (digitalRead(Encoder_PINA_Y) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(Encoder_PINB_Y) == LOW) {
      Pos_mes_Y = Pos_mes_Y - 1;         // CW
    }
    else {
      Pos_mes_Y = Pos_mes_Y + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(Encoder_PINB_Y) == HIGH) {
      Pos_mes_Y = Pos_mes_Y - 1;          // CW
    }
    else {
      Pos_mes_Y = Pos_mes_Y + 1;          // CCW
    }
  }
}
//interrupt pinB encoder
void doEncoderB_Y() {
  // look for a low-to-high on channel B
  if (digitalRead(Encoder_PINB_Y) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(Encoder_PINA_Y) == HIGH) {
      Pos_mes_Y = Pos_mes_Y - 1;         // CW
    }
    else {
      Pos_mes_Y = Pos_mes_Y + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(Encoder_PINA_Y) == LOW) {
      Pos_mes_Y = Pos_mes_Y - 1;          // CW
    }
    else {
      Pos_mes_Y = Pos_mes_Y + 1;          // CCW
    }
  }
}


/******** FONCTIONS ********/


/****  Generation consigne en rampe   ****/
float Ramp(int unsigned NbreFoisInterrupt, float periode, float vitesse_ref, float pos_ref_final,float pos_ref_depart)
{
  float pos_ref;
  pos_ref = (float)vitesse_ref*periode*NbreFoisInterrupt+pos_ref_depart ;
  
  //Fin de rampe ne doit pas dépasser la pos_ref_final
  if(vitesse_ref > 0){ //Si positif
    if(pos_ref >= pos_ref_final)
    {
      pos_ref = pos_ref_final;
      //speed feedforward nul
      f_feedforward_X = 0;
      f_feedforward_Y = 0;
    }
  }
  else if(vitesse_ref < 0){ //Si négatif
    if(pos_ref <= pos_ref_final)
    {
      pos_ref = pos_ref_final;
      //speed feedforward nul
      f_feedforward_X = 0;
      f_feedforward_Y = 0;
    }
  }
  return pos_ref;
}

/**** Conversion mm to rad ****/
float mm2rad(float dist)
{
  //Conversion de mm en rad
  // fonctionne aussi pour mm/s en rad/s
  return dist*(RAD_PER_MM);
}

/**** Conversion rad to mm ****/
float rad2mm(float dist)
{
  //Conversion de rad en mm
  // fonctionne aussi pour mm/s en rad/s
  return (dist/RAD_PER_MM);
}

/**** Mouvement moteur ****/
void MoveMoteur(int Step_Pin)
{
    digitalWriteFast(Step_Pin, HIGH);
    digitalWriteFast(Step_Pin, LOW);
}

/**** Signe d'une valeur ****/
int sign(float x)
{
    //Renvoie le signe de x
    //-1 si x < 0
    //+1 si x > 0
    //0 si x == 0
    return ((x>0)-(x<0));
}


/******** SETUP ********/


void setup() {
  // put your setup code here, to run once:

  /**** PIN ****/
    //Motors
  pinModeFast(STEP_PIN_X,OUTPUT);
  pinModeFast(DIR_PIN_X,OUTPUT);
  pinModeFast(STEP_PIN_Y,OUTPUT);
  pinModeFast(DIR_PIN_Y,OUTPUT);
    //Encoders
  pinMode(Encoder_PINA_X, INPUT);
  pinMode(Encoder_PINB_X, INPUT);
  pinMode(Encoder_PINA_Y, INPUT);
  pinMode(Encoder_PINB_Y, INPUT);
    //Limit
  pinMode(LIMIT_PIN_X,INPUT_PULLUP);
  pinMode(LIMIT_PIN_Y,INPUT_PULLUP);
    //Servo
  myservo.attach(SERVO_PIN);
  delay(15);
  myservo.write(Servo_High,255,true); // set the intial position of the servo, as fast as possible, wait until done

  //Encoder external interrupts
  PCintPort::attachInterrupt(Encoder_PINA_X, doEncoderA_X, CHANGE);
  PCintPort::attachInterrupt(Encoder_PINB_X, doEncoderB_X, CHANGE);
  PCintPort::attachInterrupt(Encoder_PINA_Y, doEncoderA_Y, CHANGE);
  PCintPort::attachInterrupt(Encoder_PINB_Y, doEncoderB_Y, CHANGE);


  /**** Initialisation position ****/
    //Déplacement du chariot jusqu'à activation des limit switchs
  digitalWriteFast(DIR_PIN_X,LOW); //Direction négative
  digitalWriteFast(DIR_PIN_Y,LOW);
      //Mouvement selon X
  while(digitalRead(LIMIT_PIN_X) == 1)
  {
    MoveMoteur(STEP_PIN_X);
    delay(1);
  }
      //Mouvement selon Y
  while(digitalRead(LIMIT_PIN_Y) == 1)
  {
    MoveMoteur(STEP_PIN_Y);
    delay(1);
  }
  delay(100);
    //Avancement pour chaque moteur à environ 5mm 
    //    => surface de travail sécuritaire de 300x380 mm^2
  int avance = 5*STEPS_PER_MM; //5mm * STEPS_PER_MM = 437.5
  digitalWriteFast(DIR_PIN_X,HIGH); //Direction négative
  digitalWriteFast(DIR_PIN_Y,HIGH);
      //Mouvement selon X
  for (int i=0;i <= avance ;i++) 
  {
    MoveMoteur(STEP_PIN_X);
    delay(1);
  }
      //Mouvement selon Y
  for (int i=0;i <= avance ;i++) //435
  {
    MoveMoteur(STEP_PIN_Y);
    delay(1);
  }
  delay(100);
    //Valeurs positions mise à zéro
  Pos_mes_X = 0;
  Pos_mes_Y = 0;
  
  
  /**** TIMER ****/
  // Initialisation du timer 2
  f_e = 5000;
  T_e = (float)1/f_e;
  // TIMER 2 for interrupt frequency 5000 Hz:
  cli(); // stop interrupts
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  // 5000 Hz 
  OCR2A = 24; // (16000000/(5000*128)) = 25 puis -1
  // CTC
  TCCR2A |= (1 << WGM21);
  // Prescaler 128
  TCCR2B |= (1 << CS22) | (1 << CS20);
  // Output Compare Match A Interrupt Enable
  TIMSK2 |= (1 << OCIE2A);
  sei(); // allow interrupts

  // Initialisation des valeurs de fréquences
  f_Controller = 500; //Hz //Fréquence contrôleur P
  T_Controller = (float) 1/f_Controller;
  f_ext = 2; //Hz //Fréquence du contrôle prédictif
  T_ext = (float) 1/f_ext;

  // Initialisation à zéro des compteurs d'interrupt
  compt_Controller = 0; 
  compt_ext = 0;
  compt_pulseMoteurY = 0;
  compt_pulseMoteurX = 0;
  

  /**** Contrôleur interne ****/
  //Kp_lim=254647; //limite stabilité pour T_e = 1/500 s
  Kp_X = 127324; //Gain proportionnel 
  Kp_Y = 127324; 

  
  /**** Communication ****/
  Serial.begin(115200); // baudrate
  delay(100);
  //Synchronisation setup
  Serial.println("a");
  a = 'b';
  while (a != 'a') //N'avance pas tant que 'a' pas reçu
  {
    a = Serial.read();
  }
  a = 'b';
  
}


/******** BOUCLE PRINCIPALE ********/


void loop() {
  // put your main code here, to run repeatedly

  // Contrôle prédictif externe
  if(compt_ext>=(f_e/f_ext)) //Communication avec Matlab
  {
     //cli(); //désactivation interrupts => peut pas car communication série utilise interrupts
    Comptage_desactive = 1; //Empeche incrémentation des compteurs pendant communication

    //Envoyer position pour Matlab
    pos_mes_X_rad = Pos_mes_X*2*pi/res_enc; //passage en radian
    pos_mes_Y_rad = Pos_mes_Y*2*pi/res_enc; //passage en radian
    ToSend = String(String(rad2mm(pos_mes_X_rad))+','+String(rad2mm(pos_mes_Y_rad))); //arrondi à deux chiffres après la virgule
    Serial.println(ToSend);
    
    //... calcul MPC ... Matlab doit renvoyer les valeurs des commandes :
      //Handshake : vérifier que buffer est vide avant de recevoir commandes
    while (a != 'a') //N'avance pas tant que 'a' pas reçu
    {
      a = Serial.read();
    }
    a = 'b'; //reset valeur de communication
    while (Serial.available()>0) //S'assurer que buffer bien vide
    {
      Serial.read(); //Vide le buffer
      //Serial.print(Serial.read()); //Vide le buffer
      //Serial.print("test");
    }
    Serial.println("a"); //Envoie que bien vide
    
      // Attente que Matlab envoie des valeurs
    while (Serial.available()<=NbreCharacters){ //15 caractères "+xx.xxx,+yy.yyy"
      //Do Absolutely Nothing until something is received over the serial port
    }
    // => lecture des valeurs
    while(Serial.available()>0)
    {
      char ch = Serial.read();
      if (ch == ',')  // comma is our separator, so move on to the next field
      {
        //Sauvegarde de la valeur
        values[fieldIndex] = ReadingSerial.toFloat();

        //Incrémentation de l'indice pour values[];
        if(fieldIndex < NUMBER_OF_FIELDS-1)
        {
          fieldIndex++;   // increment field index
        }
        ReadingSerial = ""; //reset du ReadingSerial
      }
      else if((ch >= '0' && ch <= '9')||(ch == '.')||(ch == '-'))
      {
        ReadingSerial += ch;  
      }
      else if(ch == 'Z')
      {
        Servo_activated = 1;
      }
    } 
    //Sauvegarde de la dernière valeur
    values[fieldIndex] = ReadingSerial.toFloat();
    
    ReadingSerial = "";//reset du ReadingSerial
    ToSend = "";//reset du String regroupant les positions
    fieldIndex = 0; //reset de l'indice de parcours de values[]

    if (Servo_activated == 1) //Servomoteur
    {
      if (values[0] == 1) //Position haute
      {
        myservo.write(Servo_High,255,true); // move the servo to 180, max speed, wait until done
      }
      else //Position basse
      {
        myservo.write(Servo_Low,Servo_SlowSpeed,true);  // move the servo to 180, slow speed, wait until done
      }      
      Servo_activated = 0; //reset value
      compt_ext= f_e/f_ext; //Force communication à nouveau à la prochaine loop
    }
    else //Mouvement des moteurs
    {
      //Faut passer en rad/s car en mm/s
      vitesse_ref_X = mm2rad(values[0]); 
      vitesse_ref_Y = mm2rad(values[1]);
      //Position finale de chacune des rampes de consigne
      pos_ref_final_X = pos_mes_X_rad + T_ext*vitesse_ref_X ;
      pos_ref_final_Y = pos_mes_Y_rad + T_ext*vitesse_ref_Y ;
      pos_ref_depart_X = pos_mes_X_rad;
      pos_ref_depart_Y = pos_mes_Y_rad;

      //Feedforward
      f_feedforward_X = values[0]*STEPS_PER_MM;
      f_feedforward_Y = values[1]*STEPS_PER_MM;
      
      Comptage_desactive = 0; //Réactivation comptage interrupts
      compt_ext = 0; //reset compteur lié à la boucle extérieure
      compt_time = 0; //reset compteur pour génération de rampe
    }
    memset(values, 0, sizeof(values)); //Erase old values
  }


  /**** Mouvement et contrôle interne ****/
  else
  {
    /** Contrôle interne **/
    if(compt_controle >= compt_Controller){
      compt_time+=1;
      //Get position
        //x :
          pos_ref_X = Ramp(compt_time,T_Controller,vitesse_ref_X,pos_ref_final_X,pos_ref_depart_X);
          pos_mes_X_rad = Pos_mes_X*2*pi/res_enc; //passage en radian
        //y :
          pos_ref_Y = Ramp(compt_time,T_Controller,vitesse_ref_Y,pos_ref_final_Y,pos_ref_depart_Y);
          pos_mes_Y_rad = Pos_mes_Y*2*pi/res_enc; //passage en radian
  
      //Erreur et Calcul Contrôleur
        //x :
          erreur_X = pos_ref_X - pos_mes_X_rad;
          command_freq_X = f_feedforward_X + Kp_X*erreur_X; 
        //y :
          erreur_Y = pos_ref_Y - pos_mes_Y_rad;
          command_freq_Y = f_feedforward_Y + Kp_Y*erreur_Y;
  
      //Direction
        //y :
        DIR_Y_Command = sign(command_freq_Y);
        if(DIR_Y_Command <0){ //si frequence negative
            digitalWriteFast(DIR_PIN_Y,LOW);
            command_freq_Y = - command_freq_Y;
          }
        else{ //si frequence positive 
            digitalWriteFast(DIR_PIN_Y,HIGH);
          }
        //x :
        DIR_X_Command = sign(command_freq_X);
        if(DIR_X_Command <0){
            digitalWriteFast(DIR_PIN_X,LOW);
            command_freq_X = - command_freq_X;
          }
        else{
            digitalWriteFast(DIR_PIN_X,HIGH);
          }

      //Reset compteur
        compt_controle=0;
      }

    /** Mouvement des moteurs **/
      
    //MOTEUR Y
    if(compt_pulseMoteurY >= (f_e/command_freq_Y)){
      if(abs(erreur_Y) <= (RAD_PER_STEP/2))
      {
        //erreur inférieure à la moitié d'un micropas
        //=> ne pas bouger car faire un micropas éloignera plus que si reste fixe
      }  
      else
      {
      // Mouvement moteur Y
        MoveMoteur(STEP_PIN_Y);
      //Reset compteur
        compt_pulseMoteurY=0;
      }
    }
      
    //MOTEUR X
    if(compt_pulseMoteurX >= (f_e/command_freq_X)){
      if(abs(erreur_X) <= (RAD_PER_STEP/2))
      {
        //erreur inférieure à la moitié d'un micropas
        //=> ne pas bouger car faire un micropas éloignera plus que si reste fixe
      } 
      else
      {
      // Mouvement moteur X
        MoveMoteur(STEP_PIN_X);
      //Reset compteur
        compt_pulseMoteurX=0;
      }
    }
  }
}














