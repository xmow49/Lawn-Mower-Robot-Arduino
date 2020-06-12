/*---------------------------------------------------
                    Fichier Config Pins
----------------------------------------------------*/

//-----------------Capteurs Ultrason------------------

//Capteur Gauche
#define CAPT_G_Trigers    53
#define CAPT_G_ECHO       52

//Capteur Milieu
#define CAPT_M_Trigers    51
#define CAPT_M_ECHO       50

//Capteur Droite
#define CAPT_D_Trigers    49
#define CAPT_D_ECHO       48

//-----------------L298N (MOTEURS)------------------

//Moteur Droite
#define IN1_D             22
#define IN2_D             23
#define PWM_D             9

//Moteur Gauche
#define IN3_G             24
#define IN4_G             25
#define PWM_G             10

//Moteur Hauteur Coupe
#define IN3_C             44
#define IN4_C             45
#define PWM_C             7

#define ENCODER_C1        3
#define ENCODER_C2        33

//-----------------ENDSTOP------------------

#define ENDSTOP_G       18

#define ENDSTOP_D       19

#define ENDSTOP_C       30

//-----------------GYROSCOP-----------------

#define GYRO_INIT 2
//-----------------Moteur Coupe-----------------

#define CUTING_MOTOR      28

//--------------------Config Code------------------

#define DISTANCE_OBSTACLE 20 //cm

#define VITESSE_AVENCE 200   //Valeur entre 0 et 255

#define VITESSE_RECUL 255    //Valeur entre 0 et 255

#define VITESSE_HAUTEUR_COUPE 100 //Valeur entre 0 et 255

#define MAX_VALUE_ENCODER -200 // Valeur Maximal de l'encodeur

//---------------Variables-------------


