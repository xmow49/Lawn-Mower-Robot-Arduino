#include <config.h>
#include <Arduino.h>
#include <PIDController.h>
#include <variable.h>

PIDController hauteurMoteurCoupe;

void Avencer(int vitesse)
{
    Serial.println("Avencer");
    //Moteur gauche
    digitalWrite(IN1_D, LOW);
    digitalWrite(IN2_D, HIGH);

    //Moteur droit
    digitalWrite(IN3_G, LOW);
    digitalWrite(IN4_G, HIGH);

    //Controle Vitesse
    analogWrite(PWM_D, vitesse);
    analogWrite(PWM_G, vitesse);
}

void Reculer(int vitesse)
{
    Serial.println("Reculer");
    //Moteur gauche
    digitalWrite(IN1_D, HIGH);
    digitalWrite(IN2_D, LOW);

    //Moteur droit
    digitalWrite(IN3_G, HIGH);
    digitalWrite(IN4_G, LOW);

    //Controle Vitesse
    analogWrite(PWM_D, vitesse);
    analogWrite(PWM_G, vitesse);
}

void Droite(int vitesseGauche, int vitesseDroite)
{
    Serial.println("Droite");
    //Moteur gauche
    digitalWrite(IN3_G, HIGH);
    digitalWrite(IN4_G, LOW);

    //Moteur droit
    digitalWrite(IN3_G, LOW);
    digitalWrite(IN4_G, HIGH);

    //Controle Vitesse
    analogWrite(PWM_D, vitesseDroite);
    analogWrite(PWM_G, vitesseGauche);
}

void Gauche(int vitesseGauche, int vitesseDroite)
{
    Serial.println("Gauche");
    //Moteur gauche
    digitalWrite(IN3_G, LOW);
    digitalWrite(IN4_G, HIGH);

    //Moteur droit
    digitalWrite(IN3_G, HIGH);
    digitalWrite(IN4_G, LOW);

    //Controle Vitesse
    analogWrite(PWM_D, vitesseDroite);
    analogWrite(PWM_G, vitesseGauche);
}

int ScanCaptG()
{
    digitalWrite(CAPT_G_Trigers, LOW);
    delayMicroseconds(2);
    digitalWrite(CAPT_G_Trigers, HIGH);
    delayMicroseconds(10);
    digitalWrite(CAPT_G_Trigers, LOW);
    return pulseIn(CAPT_G_ECHO, HIGH) / 58.0;
}

int ScanCaptM()
{
    digitalWrite(CAPT_M_Trigers, LOW);
    delayMicroseconds(2);
    digitalWrite(CAPT_M_Trigers, HIGH);
    delayMicroseconds(10);
    digitalWrite(CAPT_M_Trigers, LOW);
    return pulseIn(CAPT_M_ECHO, HIGH) / 58.0;
}

int ScanCaptD()
{
    digitalWrite(CAPT_D_Trigers, LOW);
    delayMicroseconds(2);
    digitalWrite(CAPT_D_Trigers, HIGH);
    delayMicroseconds(10);
    digitalWrite(CAPT_D_Trigers, LOW);
    return pulseIn(CAPT_D_ECHO, HIGH) / 58.0;
}

void changeDirection()
{
    Reculer(VITESSE_RECUL);
    delay(500);
    Droite(150, 150);
    delay(500);
}

void changeDirectionDroite()
{
    Reculer(VITESSE_RECUL);
    delay(500);
    Droite(150, 150);
    delay(500);
}
void changeDirectionGauche()
{
    Reculer(VITESSE_RECUL);
    delay(500);
    Gauche(150, 150);
    delay(500);
}

void changeDirectionMillieu()
{
    Reculer(VITESSE_RECUL);
    delay(1000);
    Gauche(150, 150);
    delay(1000);
}

void debug()
{
    Serial.print(" Gauche: ");
    Serial.println(ScanCaptG());

    Serial.print("Millieu: ");
    Serial.println(ScanCaptM());

    Serial.print(" Droite: ");
    Serial.println(ScanCaptD());
}

bool obstacleCapt(int distance)
{
    if (ScanCaptG() <= distance || ScanCaptM() <= distance || ScanCaptD() <= distance)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void obstacleEndstop()
{
    obstacle_Endstop = 1;
}



void stopHauteurCoupe()
{
    Serial.println("stopHauteurCoupe");
    digitalWrite(IN3_C, LOW);
    digitalWrite(IN4_C, LOW);

    //Controle Vitesse
    analogWrite(PWM_C, 0);
}
void decendreHauteurCoupe(int vitesse)
{
    Serial.println("Decendre Coupe");
    digitalWrite(IN3_C, HIGH);
    digitalWrite(IN4_C, LOW);

    //Controle Vitesse
    analogWrite(PWM_C, vitesse);
}
void checkHauteurCoupe()
{
    if (digitalRead(ENDSTOP_C))
    {
        stopHauteurCoupe();
        
    }
}

bool endstopCoupe(){
    return digitalRead(ENDSTOP_C);
}
void monterHauteurCoupe(int vitesse)
{
    Serial.println("Monter Coupe");
    digitalWrite(IN3_C, LOW);
    digitalWrite(IN4_C, HIGH);

    //Controle Vitesse
    analogWrite(PWM_C, vitesse);
    checkHauteurCoupe();
}
void encoder()
{
    if (digitalRead(ENCODER_C2) == HIGH)
    {
        postionEncodeur++;
    }
    else
    {
        postionEncodeur--;
    }
}

void INIT()
{
    //Capteur Ultrason
    pinMode(CAPT_G_Trigers, OUTPUT);
    pinMode(CAPT_M_Trigers, OUTPUT);
    pinMode(CAPT_D_Trigers, OUTPUT);

    pinMode(CAPT_G_ECHO, INPUT);
    pinMode(CAPT_M_ECHO, INPUT);
    pinMode(CAPT_D_ECHO, INPUT);

    //Moteurs Droite
    pinMode(IN1_D, OUTPUT);
    pinMode(IN2_D, OUTPUT);
    pinMode(PWM_D, OUTPUT);

    //Moteurs Gauche
    pinMode(IN3_G, OUTPUT);
    pinMode(IN4_G, OUTPUT);
    pinMode(PWM_D, OUTPUT);

    //Moteur Hauteur Coupe
    pinMode(IN3_C, OUTPUT);
    pinMode(IN4_C, OUTPUT);
    pinMode(PWM_C, OUTPUT);

    //ENDSTOP
    pinMode(ENDSTOP_G, INPUT_PULLUP);
    pinMode(ENDSTOP_D, INPUT_PULLUP);
    pinMode(ENDSTOP_C, INPUT);

    //Endstop Gauche
    attachInterrupt(digitalPinToInterrupt(ENDSTOP_G), obstacleEndstop, CHANGE);
    //Endstop Droite
    attachInterrupt(digitalPinToInterrupt(ENDSTOP_D), obstacleEndstop, CHANGE);

    //Ecodeur Hauteur Coupe
    pinMode(ENCODER_C1, INPUT_PULLUP);
    pinMode(ENCODER_C2, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_C1), encoder, RISING);

    hauteurMoteurCoupe.begin();
    hauteurMoteurCoupe.tune(15, 0, 2000);
    hauteurMoteurCoupe.limit(-255, 255);

    Serial.begin(9600);
    Serial.println("START OK");
}

void debug_Endstop()
{
    Serial.print("D: ");
    Serial.println(endstop_D);
    Serial.print("G: ");
    Serial.println(endstop_G);
}

void checkObstacle()
{
    if (obstacle_Endstop)
    {
        endstop_G = digitalRead(ENDSTOP_G);
        endstop_D = digitalRead(ENDSTOP_D);

        debug_Endstop();

        if (!endstop_G && !endstop_D)
        {
            changeDirectionMillieu();
        }
        else if (!endstop_G)
        {
            changeDirectionDroite();
        }
        else if (!endstop_D)
        {
            changeDirectionGauche();
        }
        else
        {
        }
        obstacle_Endstop = 0;
    }
    if (obstacleCapt(DISTANCE_OBSTACLE))
    {
        if (ScanCaptG() <= DISTANCE_OBSTACLE && ScanCaptM() <= DISTANCE_OBSTACLE && ScanCaptD() <= DISTANCE_OBSTACLE)
        {
            changeDirectionMillieu();
        }
        else if (ScanCaptG() <= DISTANCE_OBSTACLE)
        {
            changeDirectionDroite();
        }
        else if (ScanCaptM() <= DISTANCE_OBSTACLE)
        {
            changeDirectionMillieu();
        }
        else if (ScanCaptD() <= DISTANCE_OBSTACLE)
        {
            changeDirectionGauche();
        }
    }
}

void decendreDe(int d)
{
    int tmpPos = postionEncodeur;
    decendreHauteurCoupe(150);
    while (d + tmpPos > postionEncodeur)
    {
    }
    stopHauteurCoupe();
}

void setCoupePosition(int pos){
    if(postionEncodeur > pos){
        //Decendre
        decendreHauteurCoupe(VITESSE_HAUTEUR_COUPE);
        Serial.println(postionEncodeur);
        while (postionEncodeur > pos && postionEncodeur >= -330)
        {}
        stopHauteurCoupe();
    }
    else if(postionEncodeur < pos){
        //Monter
        monterHauteurCoupe(VITESSE_HAUTEUR_COUPE);
        while (postionEncodeur < pos && digitalRead(ENDSTOP_C) == 0)
        { 
        }
        stopHauteurCoupe();
        postionEncodeur = 0;

        

    }else{}
}