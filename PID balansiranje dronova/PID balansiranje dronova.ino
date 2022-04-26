// Mislav Stipić // FESB // 2020.

// PID regulator, sustav za balansiranje dronova

#include <Wire.h>           // biblioteka za komunikaciju s akcelerometrom/giroskopom  (I2C protokol)
#include <Servo.h>          // biblioteka za generiranje PWM signala

float razlikaVrijeme;    // definiranje prođenog, trenutnog vremena i predhodnog vremena
float time;
float predVrijeme;

Servo desni_motor;          // definiramo motore
Servo lijevi_motor;

/////////////////////////// PID KONSTANTE
double kp = 3.67;   //3.67
double ki = 0.004;  //0.004
double kd = 2.03;   //2.03

double brzina = 1200;     // početna brzina motora
float zeljeni_kut = 0;    // kut koji želimo postići


// definiranje variabli za akcelerometar (treba nam int16 jer akcelerometar nam daje 16 bitova podataka)
int16_t Akc_senzorX;
int16_t Akc_senzorY;
int16_t Akc_senzorZ;

// definiranje variable za giroskop (treba nam int16 jer giroskop nam daje 16 bitova podataka)
int16_t Giro_senzorY;


//pomoćne varijabla za spremanje kuta akceleracije, giroskopa i ukupnog kuta    
// za balans koristimo samo jednu os a to je u našem slučaju os X kao glavnu os --> pa računamo sa Y osi
float kutAkceleracije_Y; 
float kutGiroskopa_Y;
float ukupniKut_Y;


float rad_u_stupanj = 180 / 3.1416;      // varijabla za pretvorbu radijana u stupnjeve

float PID;                // varijabla za spremanje zbroja trenutnih vrijednosti proporcionalniog, derivacijskog i integralnog izračuna
float pwmLijevo;          // varijabla za spremanje brzine za lijevi motor
float pwmDesno;           // varijabla za spremanje brzine za desni motor
float greska_kuta;              // greška kuta položaja vage
float predhodna_greska_kuta;    // predhodna greška kuta položaja vage

// pomoćne varijable za račinanje trenutnih P I i D djelova koji se zbrajaju u varijablu PID
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;


           
// setup funkcija, nužna za rad programa, kod unutar ove funkcije se izvršava samo jedan put
void setup(){
  
  Wire.begin(); // započmi wire komunikaciju
  Wire.beginTransmission(0x68);     // započmi wire komunikaciju giroskopa i akcelerometra (I2C komunikacija)
  Wire.write(0x6B);                 
  Wire.write(0);
  Wire.endTransmission(true);
  
  
  Serial.begin(250000);     // serijska komunikacija računala i arduina (ili giroskopa ovisno što želimo gledat serijskim monitorom)
  desni_motor.attach(3);     // motor na pin 3
  lijevi_motor.attach(5);    // motor na pin 5

  time = millis();          //Počni brojanje u milisekundama

  // da bi pokrenili ploče koje upravljaju propelerima (ESC) 
  // moramo im poslati minimalnu vrijednost PWM signala kako bi ih pokrenuli
  
  lijevi_motor.writeMicroseconds(1000);     // 1000 je minimalna brzina (50% duty cycle)
  desni_motor.writeMicroseconds(1000);      
  delay(6000);     // delay za stabilan rad i sigurnost

}



//////////////////////// početak glavne petlje ///////////////////////////////////////////////////////////////////////////
void loop() {


  broji_vrijeme();  // funkcija za spremanje sadašnjeg i predhodnog vremena brojanja da bi izračunali razliku između predhodnog i sadašnjeg vremena



// AKCELEROMETAR *******************************************************************************************************************************************************
// čitaj podatke iz akcelerometra

  uzmi_podatke_iz_akcelerometra();  //funkcija koja uzima podatke iz akcelerometra i sprema ih u varijable Akc_senzorX,Y i Z


//Računamo kut pomoću Ojlerove jednađžbe///*/
    
    // djelimo dobivene vrijednosti akceleracija sa 16384.0 da bi dobili pravu vrijednost u m/sˇ2
    // jer nam tako kaže datasheet.pdf akcelerometra/giroskopa

      // za balans koristimo samo jednu os a to je u našem slučaju os X kao glavnu os pa računamo sa Y osi
      // Y vrijednost akceleracije
     kutAkceleracije_Y = atan(-1 * (Akc_senzorX / 16384.0) / sqrt( pow(( Akc_senzorY / 16384.0), 2 ) + pow(( Akc_senzorZ / 16384.0 ), 2 ))) * rad_u_stupanj;







// GIROSKOP ****************************************************************************************************************************************************

 
// sada čitamo vrijednosti sa giroskopa kao i kod akceleracija i spremamo ih u pomoćne varijable

  uzmi_podatke_iz_giroskopa(); 

   // ukupni kut nam se kreće između -20 stupnjeva i 90 stupnjeva (u našem modelu)
   // za balans koristimo samo jednu os a to je u našem slučaju os X kao glavnu os pa računamo sa Y osi
   // kut po Y u stupnjevima
   ukupniKut_Y = 0.98 *(ukupniKut_Y + kutGiroskopa_Y * razlikaVrijeme) + 0.02 * kutAkceleracije_Y;
   



//// DIO ZA PID REGIULATOR *******************************************************************************************************************************


// za balans koristimo samo jednu os a to je u našem slučaju os X kao glavnu os pa računamo sa Y osi
// prvo računamo grešku između trenutnog kuta i željenog kuta

  greska_kuta = ukupniKut_Y - zeljeni_kut;
    

// proporcionalni dio je samo konstanta
  pid_p = kp * greska_kuta;



// integralni dio djeluje samo kad smo blizu željenog kuta, zato imamo -3 i 3 stupnja
// da bi integrirali samo moramo zbrojiti predhodnu vrijednost integrala 
// sa integralnom konstantom koju množimo s greškom

  if(-3 <greska_kuta <3){
   pid_i = pid_i + (ki * greska_kuta);  
}


// deriviramo tako da pomnožimo derivacijsku konstantu sa 
//razlikom trenutne greške i predhodne greške i podjelimo 
//sve sa vremenom koje protekne između te dvije greške
  pid_d = kd * ((greska_kuta - predhodna_greska_kuta) / razlikaVrijeme);



// na kraju zbrojimo proporcionalni integralni i derivacijski dio da bi dobili PID
  PID = pid_p + pid_i + pid_d;




// funkcija za postavljanje ograničenja varijable PID kako nebi oštetila ESC-ove
  fun_za_zastitu_ESC_1(); 



// funkcija za proračun brzine svakog motora ovisno o kutu položaja vage i akceleracije vage
  izracunaj_potrebnu_trenutnu_brzinu_motora(); 




// opet moramo osigurati da ne prelazimo dopuštene brzine vrtnje motora koje bi nam dao 
// PID regulator da se balansira pa opet postavljamo granice posebno za motore
// (osiguravamo da se ne prelazi maksimalna i minimalna brzina motora)

  fun_za_zastitu_ESC_2();



// koristimo servo funkcije da bi generirali PWM signal već predhodno izračunatog trajanja "on" dijela
  lijevi_motor.writeMicroseconds(pwmLijevo);
  desni_motor.writeMicroseconds(pwmDesno);

//spremi trenutnu pogrešku kao predhodnu za novi loop proračuna
  predhodna_greska_kuta = greska_kuta; 




}
//////////////////////// kraj glavne petlje ///////////////////////////////////////////////////////////////////////////




void uzmi_podatke_iz_akcelerometra(){
  
     Wire.beginTransmission(0x68);                    // adresa ploče sa senzorima
     Wire.write(0x3B);                                // zatraži 0x3B register (prvi registar) adresu u kojem je spremljena akceleracija
     Wire.endTransmission(false);                     // ne prekidaj uzimanje podataka
     Wire.requestFrom(0x68,6,true);                   // zatraži vrijednosti x,y,z akceleracije (2 bajta puta 3 vrijednosti x,y,z)

   // zbroji 2 bajta od svakog para da bi dobili y vrijednosti akceleracije (binarno zbrajanje)
     Akc_senzorX = Wire.read() << 8 | Wire.read();   //svaka vrijednost uzima po 2 registra i zbrajaju se binarno
     Akc_senzorY = Wire.read() << 8 | Wire.read();
     Akc_senzorZ = Wire.read() << 8 | Wire.read();

}




void uzmi_podatke_iz_giroskopa(){

  
   // adresa giroskopa počinje s 0x43. potrebno nam je samo X i Y vrijednost jer ne koristimo Z kod giroskopa
    
   Wire.beginTransmission(0x68);    // adresa ploče sa senzorima
   Wire.write(0x43);                // adresa giroskopa je 0x43.
   Wire.endTransmission(false);     // ne prekidaj uzimanje podataka
   Wire.requestFrom(0x68,4,true);   // uzimamo samo 4 registra

   Giro_senzorY = Wire.read() << 8 | Wire.read();   // šiftamo registre i zbrajamo ih (binarno zbrajanje)
   Giro_senzorY = Wire.read() << 8 | Wire.read();  

   
   //da bi imali podatke iz giroskopa u  °/sek prvo moramo podjelit dobivene vrijednosti s 131
   //jer nam tako kaže datasheet.pdf
   // spremi prave vrijednosti u varijablu kutGiroskopa_Y

   // Y vrijednost (koristimo samo Y os, a X os nam služi kao glavna os)
   kutGiroskopa_Y = Giro_senzorY / 131.0;
}




void izracunaj_potrebnu_trenutnu_brzinu_motora(){
  
// na kraju računamo dužinu trajanja "on" djela PWM signala tako da zbrojimo brzinu s dobivenim PID dijelom
  pwmLijevo = brzina + PID;
  pwmDesno = brzina - PID;

}




void fun_za_zastitu_ESC_1(){

  // minimalna vrijednost PWM signala je 1000us a maksimalna je 2000. PID oscilira između -1000 i 1000
// ovim ispod uvjetima osiguravamo da PID ostane između -1000 i 1000 da nebi oštetili drajvere za motore

  if(PID < -1000){
   PID = -1000;
  }
  if(PID > 1000){
   PID = 1000;
  }
  
}



void fun_za_zastitu_ESC_2(){

// osiguravamo granice brzine desnog motora
  if(pwmDesno < 1000){
   pwmDesno = 1000;
  }

  if(pwmDesno > 2000){
   pwmDesno = 2000;
  }


// osiguravamo granice brzine lijevog motora
  if(pwmLijevo < 1000){
   pwmLijevo = 1000;
  }
  if(pwmLijevo > 2000){
   pwmLijevo = 2000;
  }
  
}



void broji_vrijeme(){

    predVrijeme = time;                               // prvo spremamo predvrijeme kako bi mogli koristiti vremensku razliku u sljedećim proračunima
    time = millis();                                  // pravo vrijeme
    razlikaVrijeme = (time - predVrijeme) / 1000;     // djelimo s 1000 kako bi nastavili radit sa sek a ne ms
}
