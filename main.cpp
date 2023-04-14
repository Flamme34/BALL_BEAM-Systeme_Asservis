#include "mbed.h"

// Pin definitions
PwmOut servo(p21);
DigitalOut trigger(p12);
DigitalIn echo(p13);


#define MAX 1300.0
#define MIN 900.0

// variables globales
float distance_voulue = 20; // Valeur de consigne de la distance (en cm)

//Servo-moteur
float servo_position = 1500; // Position initiale du servo-moteur (50%)

//Gain PID
float kp = -8.45; // Coefficient proportionnel du contrÃ´leur PID
float ki = -2.8; // Coefficient intÃ©gral du contrÃ´leur PID (-5)
float kd = -0.95; // Coefficient dÃ©rivÃ© du contrÃ´leur PID (-2)

//Erreur
float erreur_integrale = 0; // Erreur intÃ©grale initiale
float derniere_erreur = 0; // DerniÃ¨re erreur initiale

// Constants
const int MAX_DISTANCE = 38; // Distance max pour le HC-SR04

// Variables
float last_error = 0;
float integral = 0;
float distance_balle = 0;
float dt = 0.02;

//Calcul moyenne
float moyenne_de_mesure[5] = {};
float moyenne;

// fonction de delais en millisecondes
void wait_ms(int x)
{
	for(int i=0; i<1000; i++)
	{
		wait_us(x);
	}
}


//fonction de mesure
float mesure_distance() {
    trigger = 1;     //activation du trigger
    wait_us(10);     //attente pour bel envoi
    trigger = 0;     //arrêt de l'envoi
    while(echo == 0);//Temps que echo ne répond pas le timer continu
    Timer t;
    t.start();
    while(echo == 1 && t.read_us() < 10000); //On arrête le timer quand echo répond
    t.stop();
    distance_balle = t.read_us() / 58.0;  //conversion en distance
    if(distance_balle > MAX_DISTANCE) {
        distance_balle = MAX_DISTANCE;
    }
    return distance_balle;
}

//fonction principale
int main() 
{
    servo.period(0.02);
    int index = 0; //Mise de l'index à 0 pour calcul de la moyenne
    
    while (1) 
    {
        moyenne_de_mesure[index] =  mesure_distance(); //Capture de la distance avec la balle
        moyenne = 0; //Mise à 0 de la moyenne
        for (int i=0; i<5; i++){ //Remplir le tableau de la moyenne avec 5 mesure
            moyenne = moyenne + moyenne_de_mesure[i];
        }
        moyenne = moyenne/5; //calcul de la moyenne

        float erreur = distance_voulue - moyenne; // Calcul de l'erreur
        erreur_integrale += erreur * dt; // Ajout de l'erreur Ã  l'erreur intégrale
        float error_derivative = (erreur - derniere_erreur)/dt; // Calcul de la dérivée de l'erreur

        float output_p = kp * erreur; //calcul de la sortie proportionnelle
        float output_i = ki * erreur_integrale; //calcul de la sortie intégrale
        float output_d = kd * error_derivative; //calcul de la sortie dérivée
        output_d = output_d > 100 ? 100:output_d; //Range maximale pour la sortie dérivée
        output_d = output_d < -100 ? -100:output_d; //Range mnimale pour la sortie dérivée
        float output = 1150 + output_p + output_i + output_d; // Calcul de la sortie
        derniere_erreur = erreur; // Mise Ã  jour de la dernière erreur
        output = output > MAX ? MAX:output; // Range maximale de la sortie
        output = output < MIN ? MIN:output; // Range minimale de la sortie
        servo.pulsewidth_us(output); // Écriture de la position du servo-moteur
        printf("mes : %6.2f , moy : %6.2f, sp : %6.2f, si : %6.2f, sd : %6.2f \n\r",distance_balle,moyenne,output_p,output_i,output_d); //affichage de variables

        index++;
        if (index >= 5) {
            index = 0;
        }
        // Boucle infinie
    }
}
