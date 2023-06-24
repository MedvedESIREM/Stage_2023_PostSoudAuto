/* TOM DELPY - 15/06/2023
 Projet : Poste de soudure Automatique - APPLICATION SJMeca

 BUT : Controler une plaque sur une vis avec un moteur pas à pas à partir de plusieurs modes
       ainsi que de la commande d'un systeme de soudure (gaz + pointe)
  
  MATERIEL : Ecran (affichage) : Nextion (Intelligent, 5 pouces)
             Microcontroleur (Arduino) : PLC IndustrialShields, ARDBOX FAMILY (analog HF)

 NOTE : Affichage ralentit code -> taux de rafraichissement haut.
        (VITESSES ENTRE [0,5000] mm/min SAUF AVEC X+ et X- : [0,~3200])
        
        Si la communication Serial TTL ne fonctionne plus apres une modification du programme,
        c'est probablement a cause d'une surcharge du Serial (avec trop de print, ou un programme trop gros)

        Le code a deja ete bien optimise donc bon courage si vous etes ammene encore l'optimiser
        (piste : la librairie "EasyNextionLibrary" n'est pas obligatoire 
          mais facilite BEAUCOUP la transmission : Nextion -> Arduino)

        Les constantes ou variables locales diminuent le temps de boucle et allegent le programme
*/

// VARIABLES MODIFIABLES PAR UTILISATEUR_______________________________________________________
// CYCLE SOUDURE :
float A;  // distance ABSOLUE 1er point de soudure (mm)     [ABSOLUE par rapport à HOME]
float B;  // distance ABSOLUE 2eme point de soudure (mm)
float C;  // distance ABSOLUE point de soudure finale (mm)
float t1; // temporisation apres gaz (sec)
float t2; // temporisation apres soudure (sec)
float V1; // Vitesse de deplacement hors soudure (mm/min) [0, 4000]mm/min
float V2; // Vitesse de deplacement en soudure (mm/min) [0, 4000]mm/min
//________________________________________________________
// AUTRES PARAMETRES :
float limit_dpct;         // Limite de deplacement (mm)
float limit_v;            // Limite de vitesse (mm/min) [0, 4000]mm/min
float V_manu;             // ~Vitesse approximative de deplacement en manuel (mm/min) [0, 4000]mm/min
float V_home;             // ~Vitesse approximative de deplacement en mode home (mm/min) [0, 4000]mm/min
float count_rate;         // Taux de rafraichissement de l'ecran (mm/affichage)
float pas_vis;            // Pas de la vis à bille (mm/tr)
float MOTOR_STEPS; // Configuration du driver/moteur en (pas/tr)
//________________________________________________________
// Liste d'adresses des parametres
float* params_a[]={&A,&B,&C,&t1,&t2,&V1,&V2,&limit_dpct,&limit_v,&V_manu,&V_home,&count_rate,&pas_vis,&MOTOR_STEPS};
// Liste des noms des zones de texte des parametres sur Nextion
const String list_txt[]={"tA","tB","tC","t1","t2","tV1","tV2","tLD","tLV","tVM","tVH","tR","tP","tD"};
//_____________________________________________________________________________________________

// INCLUDE
#include <Nextion.h>              // NEXTION DISPLAY
#include "EasyNextionLibrary.h"   // pour recevoir du txt de Nextion
#include <EEPROM.h>               // pour enregistrer les parametres
#include "IndustrialShields.h"    // PLC INDUSTRIAL SHIELDS

SoftwareSerial HMISerial(14, 15);  // Rx (MISO), Tx (SCK)
EasyNex myNex(HMISerial);

// PINS
const int voyant_b = Q0_0;      // LED VERTE
const int act_soudure = Q0_1;   // Commande activation soudure
const int ouv_gaz = Q0_2;       // Commande ouverture du gaz
const int fdc = I0_1;           // Detecteur fin de course
const int driverPUL = Q0_3;     // PUL
const int driverDIR = Q0_4;     // DIR (LOW = X+ / HIGH = X-)

// Declaration des BPs de NEXTION (page id, component id, component name)
NexButton BP_AUTO = NexButton(5, 2, "BP_AUTO");
NexButton BP_HOME = NexButton(5, 3, "BP_HOME");
NexButton BP_A = NexButton(5, 16, "BP_A");
NexButton BP_P = NexButton(5, 4, "BP_P");
bool etat_BPP = false;
NexButton BP_M = NexButton(5, 5, "BP_M");
bool etat_BPM = false;
NexButton BP_VALID = NexButton(3, 55, "BP_VALID");

// LISTE DES BPs NEXTION
NexTouch *nex_listen_list[] =
{
  &BP_AUTO,
  &BP_HOME,
  &BP_A,
  &BP_P,
  &BP_M,
  &BP_VALID,
  NULL
};

// VARIABLES A NE PAS CHANGER PAR L'UTILISATEUR
int etat_fdc;        // etat du detecteur fdc
float pos = 0;       // position en mm
float time_step_m;   // temps entre chaque impulsions pour moteur (micro sec) (EN MANUEL)
float time_step_h;   // temps entre chaque impulsions pour moteur (micro sec) (EN HOME)
float time_step_a;   // temps entre chaque impulsions pour moteur (micro sec) (EN A)
float time_step_v1;  // temps entre chaque impulsions pour moteur (micro sec) (EN V1)
float time_step_v2;  // temps entre chaque impulsions pour moteur (micro sec) (EN V2)
float distance_1pas; // distance de la plaque après un pas du moteur (mm) 

// SETUP
void setup(void) {
  HMISerial.begin(9600);  // Debut de la communication serial avec le baud=9600
  
  // Recuperation de la valeur des parametres dans la memoire EEPROM
  for(int i=0; i<14; i++){
    *params_a[i]=EEPROM.get(10*(i+1), *params_a[i]);
  }

  // Init I/O
  // Sorties
  pinMode(driverPUL, OUTPUT);
  pinMode(driverDIR, OUTPUT);
  pinMode(act_soudure, OUTPUT);
  pinMode(ouv_gaz, OUTPUT);
  pinMode(voyant_b, OUTPUT);
  // Entrees
  pinMode(fdc, INPUT);

  // SETUP pour NEXTION
  myNex.begin(9600);
  delay(500);
  nexInit();
  
  // Attribution des fonctions aux BPs de Nextion
  // |Note| Push : Appuyer | Pop : Relacher
  BP_P.attachPop(BP_P_pop);
  BP_P.attachPush(BP_P_push);
  BP_M.attachPop(BP_M_pop);
  BP_M.attachPush(BP_M_push);
  BP_A.attachPush(BP_A_push);
  BP_AUTO.attachPush(BP_AUTO_push);
  BP_HOME.attachPush(BP_HOME_push);
  BP_VALID.attachPush(BP_VALID_push);

  //  Sync avec le NEXTION
  String wait_init = myNex.readStr("INIT.wait_init.txt");
  while(wait_init!="true"){
    wait_init = myNex.readStr("INIT.wait_init.txt");
    delay(500);
  }

  // ENVOI des parametres sur l'ecran Nextion
  for(int i=0; i<14; i++){
    if (i<5){
      envoi_nextion("p3."+list_txt[i], "txt", String(*params_a[i]));
    }
    else{
      envoi_nextion("p3."+list_txt[i], "txt", String(long(*params_a[i])));
    }
  }

  change_page("p3"); // chgt de page

  // CALCULS
  calculs();
  digitalWrite(driverDIR, LOW); // X-
 
}
//________________________________________________________________________________________________________

// LOOP
void loop(void) {
  // Lire les etats des BP + fdc
  nexLoop(nex_listen_list);
  etat_fdc = digitalRead(fdc);
  digitalWrite(voyant_b, LOW);  // VOYANT ON
  digitalWrite(driverDIR, LOW); // X-
}

//________________________________________________________________________________________________________
// FONCTIONS

// CALCULS____________________________________________
// Calcul de la vitesse en mode X+ et X-
// ARDUINO
float calcul_vitesse(long vitesse){
  // Vitesse entre 0 et 2000
  if (vitesse <= 2000){ 
    vitesse = 0.0008*pow(vitesse,2)+0.3368*vitesse+37.446;
  }
  // Vitesse entre 2000 et 3000
  else if (vitesse <= 3000){ 
    vitesse = 0.0308*pow(vitesse,2)-136.49*vitesse+155941;
  }
  // Vitesse entre 3000 et 4000 == ne fonctionne pas bien
  else { 
    vitesse = 1000000*17*exp(0.0024*vitesse);
  }
  return 30*pow(10,6)*(distance_1pas/(vitesse*1.4));
}
// Calcul de la vitesse en mode AUTO, HOME et A
float calcul_vitesse_auto(long vitesse){
  // Vitesse entre 0 et 500
  if (vitesse < 500){ 
    return 1318*exp(-0.003*vitesse)+25;
  }
  // Vitesse entre 500 et 1000
  else if (vitesse < 1000){ 
    return 750*exp(-0.002*vitesse)+45;
  }
  // Vitesse entre 1000 et 2000
  else if (vitesse < 2000){ 
    return  357.66*exp(pow(9,-4)*vitesse);
  }
  // Vitesse entre 2000 et 3000
  else if (vitesse < 3000){ 
    return -0.0338*vitesse + 124.4;
  }
  // Vitesse entre 3000 et 4000
  else if (vitesse < 4000){ 
    return -0.0172*vitesse + 74.6;
  }
  // Vitesse entre 4000 et 5000
  else { 
    return -0.005*vitesse + 26;
  }
}
// Fonction qui realise quelques calculs utiles
void calculs(){
  float pas_vis_c = pas_vis;
  distance_1pas = pas_vis_c*1.0/MOTOR_STEPS*1.0;
  
// Conversion pour taux de rafraichissement de l'ecran (mm/affichage)
  count_rate = count_rate/distance_1pas; 

  // Calculs des nouvelles tempos pour avoir les bonnes vitesses
  time_step_v1 = calcul_vitesse_auto(V1);
  time_step_v2 = calcul_vitesse_auto(V2);
  time_step_a = calcul_vitesse_auto(V_manu);
  time_step_m = calcul_vitesse(V_manu);
  time_step_h = calcul_vitesse_auto(V_home);
}
//__________________________________________________

// Mode HOME________________________________________
void mode_home(){
  // Creation de constante/variable LOCALE qui diminue le temps de boucle
  float distance_1pas_c = distance_1pas;
  double time_step_h_c = time_step_h;
  int count_rate_c = count_rate;
  int count_aff = 0;
  const float true_count_rate_c = count_rate_c*distance_1pas_c;
  digitalWrite(voyant_b, HIGH);
  envoi_nextion("p4.t_etat2", "txt", "HOME");
  etat_fdc = digitalRead(fdc);
  digitalWrite(driverDIR, LOW); // X-
  // 1ere phase : retour à fdc
  while (etat_fdc != HIGH){
    etat_fdc = digitalRead(fdc);
    avance_moteur(time_step_h_c);
    // Affichage de la position
    count_aff++;
    if (count_aff >= count_rate_c){
      pos = pos - true_count_rate_c;
      display_pos(pos);
      count_aff = 0;
    }
    delayMicroseconds(8);
  }
  digitalWrite(driverDIR, HIGH); // X+
  // 2eme phase : Avance de 5mm vers +
  for(int i = 0; i < MOTOR_STEPS; i++){
    avance_moteur(time_step_h_c);
  }
  // fin de mode home
  etat_fdc = false;
  pos = 0;
  envoi_nextion("p4.t_etat2", "txt", ""); // on reinitialise le text "t_etat2" sur Nextion
  display_pos(pos);
  
}
// ___________________________________________________

// Afficher la position sur ecran_____________________
void display_pos(float position){
  envoi_nextion("p4.position_txt", "txt", String(position));
}
// ___________________________________________________

// DEMARAGE MOTEUR____________________________________
void avance_moteur(double time_b_step){
  double time_b_step_c = time_b_step;
  digitalWrite(driverPUL, HIGH);    // PUL HAUT
  delayMicroseconds(time_b_step_c); // tempo qui change la vitesse du moteur
  digitalWrite(driverPUL,LOW);      // PUL BAS
}
// ___________________________________________________

// DEMARAGE GAZ + TORCHE______________________________
void outil_on(){
  digitalWrite(ouv_gaz, HIGH);     // OUVERTURE GAZ
  delay(t1*1000);                  // DELAIS T1
  digitalWrite(act_soudure, HIGH); // ACTIVATION SOUDURE
  delay(t2*1000);                  // DELAIS T2
}
// ___________________________________________________

// DEMARAGE GAZ + TORCHE______________________________
void outil_off(){
  digitalWrite(act_soudure, LOW); // FERMETURE SOUDURE
  digitalWrite(ouv_gaz, LOW);     // FERMETURE GAZ
  delay(t1*1000);                 // DELAIS T1
}
// ___________________________________________________

// FONCTIONS DES BP DE NEXTION_______________________

// PUSH X+
void BP_P_push(void *ptr){
  // Creation de constante/variable LOCALE qui diminue le temps de boucle
  const float distance_1pas_c = distance_1pas;
  const float limit_pas = limit_dpct-distance_1pas_c;
  double time_step_m_c = time_step_m;
  const int count_rate_c = count_rate;
  int count_aff = 0;
  // MANUEL X+
  if (pos < limit_pas){
    digitalWrite(driverDIR, HIGH); // X+
    pos = pos + distance_1pas_c;
    digitalWrite(voyant_b, HIGH);
  }
  etat_BPP=true;
  while (pos < limit_pas && etat_BPP==true){
    nexLoop(nex_listen_list); // RALENTIT ENORMEMENT LA BOUCLE
    pos = pos + distance_1pas_c;
    avance_moteur(time_step_m_c);
    count_aff++;
    // AFFICHAGE DE LA POSITION
    if (count_aff >= count_rate_c){
      display_pos(pos);
      count_aff = 0;
    }
  }
}

// POP X+
void BP_P_pop(void *ptr){
  etat_BPP=false;
  display_pos(pos);
}

// PUSH X-
void BP_M_push(void *ptr){
  // Creation de constante LOCALE qui diminue le temps de boucle
  const float distance_1pas_c = distance_1pas;
  double time_step_m_c = time_step_m;
  const int count_rate_c = count_rate;
  int count_aff = 0;
  // MANUEL -
  if (pos > distance_1pas_c){
    digitalWrite(driverDIR, LOW); // X-
    pos = pos - distance_1pas_c;
    digitalWrite(voyant_b, HIGH);
  }
  etat_BPM=true;
  while (pos > pow(10, -7) && etat_BPM==true){
    nexLoop(nex_listen_list); // RALENTIT ENORMEMENT LA BOUCLE
    pos = pos - distance_1pas_c;
    avance_moteur(time_step_m_c);
    count_aff++;
    // AFFICHAGE DE LA POSITION
    if (count_aff >= count_rate_c){
      display_pos(pos);
      count_aff = 0;
    }
  }
}
// POP X-
void BP_M_pop(void *ptr){
  etat_BPM=false;
  display_pos(pos);
}

// PUSH A
void BP_A_push(void *ptr){
  delay(100); // attente pour synchro avec Nextion
  float pos_a = myNex.readNumber("posm.val");
  // TEST LIMITE DEPLACEMENT
  if (pos_a > limit_dpct){
    // PROBLEME HORS LIMITE
    envoi_nextion("p4.error_s", "en", "1");
    envoi_nextion("p4.posm", "pco", "63488");

  }
  else{
    // SINON RAS
    // Init de variables locales
    float point_pas = pos_a/distance_1pas*1.0;
    float pos_c = pos/distance_1pas*1.0;
    double time_step_a_c = time_step_a;
    float steppy;
    const int count_rate_c = count_rate;
    const float true_count_rate = count_rate_c*distance_1pas;
    int count_aff = 0;
    int way;
    envoi_nextion("t_etat", "txt", "A");
    digitalWrite(voyant_b, HIGH);
    // Test pour savoir si le nouveau point est plus grand ou plus petit que la position actuelle
    if (point_pas > pos_c){
      digitalWrite(driverDIR, HIGH); // X+
      steppy = point_pas-pos_c;
      way = 1;
    }
    else{
      digitalWrite(driverDIR, LOW); // X-
      steppy = pos_c-point_pas;
      way = -1;
    }
    for (float i=0; i<steppy; i++){
      avance_moteur(time_step_a_c);
      count_aff++;
      // AFFICHAGE DE LA POSITION
      if (count_aff >= count_rate_c){
        pos = pos + 1.0*way*true_count_rate;
        display_pos(pos);
        count_aff = 0;
      }
    }
    // FIN DU MODE A
    envoi_nextion("t_etat", "txt", "");
    pos = pos_a;
    display_pos(pos);
  }
}

// Deplacer la fleche sur le nextion
void moove_arrow(int x){
  envoi_nextion("p4_DEPCYC.arrow", "x", String(x));
}
// PUSH START (CYCLE SOUDURE AUTO)
void BP_AUTO_push(void *ptr){
  // Init de variables locales
  double time_step_v1_c = time_step_v1;
  double time_step_v2_c = time_step_v2;
  float A_pas = A/distance_1pas*1.0;
  float B_pas = B/distance_1pas*1.0;
  float C_pas = C/distance_1pas*1.0;
  float steppyAB = B_pas - A_pas;
  float steppyBC = C_pas - C_pas;
  change_page("p4_DEPCYC");
  // Active la phase de HOME
  mode_home();
  // DEBUT DE CYCLE
  // DEPLACEMENT VERS A (hors soudure)
  digitalWrite(driverDIR, HIGH); // X+
  for (float i=0; i<A_pas; i++){
      avance_moteur(time_step_v1_c);
    }
  int arrow;
  moove_arrow(arrow=226);
  // DEMARAGE EXT
  outil_on();
  // DEPLACEMENT DE A VERS B (soudure)
  for (float i=0; i<steppyAB; i++){
      avance_moteur(time_step_v2_c);
    }
  moove_arrow(arrow=376);
  // ARRET EXT
  outil_off();
  // DEPLACEMENT DE B VERS C (hors soudure)
  for (float i=0; i<steppyBC; i++){
      avance_moteur(time_step_v1_c);
    }
  moove_arrow(arrow=522);
  // DEMARAGE EXT
  outil_on();
  // DEPLACEMENT DE C VERS B (soudure)
  digitalWrite(driverDIR, LOW); // X-
  for (float i=0; i<steppyBC; i++){
      avance_moteur(time_step_v2_c);
    }
  moove_arrow(arrow=670);
  // ARRET EXT
  outil_off();
  // Active la phase de HOME
  mode_home();
  // fin du mode start (fin de cycle)
  moove_arrow(arrow=70);
  
  change_page("p4");
}

// PUSH HOME
void BP_HOME_push(void *ptr){
  mode_home();
}

// PUSH VALIDATE
void BP_VALID_push(void *ptr){
  delay(300); // attente pour synchro avec Nextion

  // On envoie les parametres de Nextion sur Arduino
  float limit_dpct_m=myNex.readStr("tLD.txt").toFloat();
  String test_lv = myNex.readStr("t0.txt");
  float Am = myNex.readStr("tA.txt").toFloat();
  float Bm = myNex.readStr("tB.txt").toFloat();
  float Cm = myNex.readStr("tC.txt").toFloat();

  // TEST DES LIMITES POUR LES POINTS A, B, C
  // (les limites de vitesses sont déjà faites)
  if (Am>limit_dpct_m || Bm>limit_dpct_m || Cm>limit_dpct_m){
    // ERROR
    if (Am>limit_dpct_m){ // erreur sur A
      envoi_nextion("p3.tA", "bco", "61571");   // change le fond du text de A sur Nextion en rouge
    }
    if (Bm>limit_dpct_m){ // erreur sur B
      envoi_nextion("p3.tB", "bco", "61571");   // change le fond du text de B sur Nextion en rouge
    }
    if (Cm>limit_dpct_m){ // erreur sur C
      envoi_nextion("p3.tC", "bco", "61571");   // change le fond du text de C sur Nextion en rouge
    }
    envoi_nextion("p3.t3", "txt", "PB*");       // affiche "PB*" sur Nextion
    envoi_nextion("p3.border", "aph", "0");     // affiche le bandeau sur Nextion
    envoi_nextion("p3.validbloc", "val", "0");  // desactive le parametre "validbloc"
    envoi_nextion("p3.validp", "txt", "");      // reinitialise le parametre "validp"
    envoi_nextion("p3.error_s", "en", "1");     // active le son "error"
  }
  else if (test_lv == "PB*"){
    envoi_nextion("p3.error_s", "en", "1");     // active le son "error"
  }
  else {
    // RAS !
    for(int i=0; i<14; i++){
      *params_a[i]=myNex.readStr(list_txt[i]+".txt").toFloat(); // lire les parametres sur l'ecran Nextion
    }

    // Enregistrement des valeurs
    for(int i=0; i<14; i++){
      EEPROM.put(10*(i+1), *params_a[i]); // mettre les valeurs des parametres sur le memoire EEPROM
    }
    
    // Re-calcul des valeurs
    calculs();
    // changement de page
    String page = myNex.readStr("p3.validp.txt");
    envoi_nextion(page+".validate_s", "en", "1");
    change_page(page);
  }
  
}
// ___________________________________________________

// OBLIGATOIRE pour communiquer avec le Nextion
void endNextionCommand(){
  HMISerial.write(0xff);
  HMISerial.write(0xff);
  HMISerial.write(0xff);
}
// CHANGER DE PAGE
void change_page(String page){
  endNextionCommand();
  HMISerial.print("page "+page);
  endNextionCommand();
}
// CHANGER LA/LE VALEUR/TEXT D'UN ELEMENT SUR NEXTION
void envoi_nextion(String text, String param, String pi){
  String quote = "";
  // Si le parametre à changer est "txt"
  if (param == "txt"){
    quote = "\"";
  }
  endNextionCommand();
  HMISerial.print(text+"."+param+"="+quote+pi+quote);
  endNextionCommand();
}
