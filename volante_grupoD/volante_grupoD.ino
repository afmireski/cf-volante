#define ATUA_CW PB4 // Sentido horário
#define ATUA_CCW PB5 // HIGH aqui gira CCW Anti-Horário
#define ATUA_STRENGTH PB3

#define ROTARY_ENC_A 6
#define ROTARY_ENC_B 7
#define ROTARY_ENC_PCINT_A PCINT22
#define ROTARY_ENC_PCINT_B PCINT23
#define ROTARY_ENC_PCINT_AB_IE PCIE2

#define POS_SENSOR PB2 // switch (absolute position)
#define GP_BUTTON PB1 // general purpose button
#define GP_BUTTON_GND PB0

#define GPB (1 << GP_BUTTON)

#define IN_DISTANCE 600 // Distância da entrada da chave até o ponto de origem
#define OUT_DISTANCE 500 // Distância da saída da chave até o ponto de origem
#define IN_STOP 600 // Distância da entrada da chave até o ponto de origem
#define OUT_STOP 10 // Distância da saída da chave até o ponto de origem
#define STOP_DELTA 400 // A distância entre um ponto e o centro onde o volante deve começar a parar de forma a se alinhar com a origem, compensando a inércia
#define LOG true
#define SPEED 160


#include "Rotary.h"
volatile long count = 0; // encoder_rotativo = posicao relativa depois de ligado
volatile bool absolute_sw = true; // chave de posicao do volante ativa?
volatile bool last_sw = absolute_sw;

Rotary r = Rotary(ROTARY_ENC_A, ROTARY_ENC_B);


// Habilita  Interrupção
void initPWM() {
  OCR2A = 0;
  TCCR2A = (1<<WGM20);
  TCCR2B = (1<<CS21);
  TCCR2A |= (1<<COM2A1);
}

// Limita a energia
void setPWM(unsigned char val) {
  OCR2A = val<200?val:200;
}

// Diminui a velocidade do motor
void Stop() {
  PORTB |= (1<<ATUA_CW);
  PORTB |= (1<<ATUA_CCW);
}

// Desliga o PWM
void Idle() {
  setPWM(0);
  PORTB &= ~(1<<ATUA_CW);
  PORTB &= ~(1<<ATUA_CCW);
}

// Liga o PWM e move o volante
void Move(unsigned char power, bool cw = true) {
  if (power == 0) // 0 a 200
    Idle();
  else {
    if (cw) { // Um horário e o outro anti-horário
      PORTB |= (1<<ATUA_CW);
      PORTB &= ~(1<<ATUA_CCW);
    } else {
      PORTB &= ~(1<<ATUA_CW);
      PORTB |= (1<<ATUA_CCW);
    }
    setPWM(power);
  }
}

void setup() {
  Serial.begin(115200);
  r.begin(true);
  PCICR |= (1 << ROTARY_ENC_PCINT_AB_IE);
  PCMSK2 |= (1 << ROTARY_ENC_PCINT_A) | (1 << ROTARY_ENC_PCINT_B);

  DDRB |= (1<<GP_BUTTON_GND);
  DDRB &= ~((1<<GP_BUTTON)|(1<<POS_SENSOR));
  DDRB |= (1<<ATUA_CW)|(1<<ATUA_CCW)|(1<<ATUA_STRENGTH);

  PORTB |= (1<<POS_SENSOR);
  PORTB &= ~(1<<GP_BUTTON_GND);
  PORTB |= (1<<GP_BUTTON);
  PORTB &= ~(1<<ATUA_CW);
  PORTB &= ~(1<<ATUA_CCW);

  initPWM();
  sei();

  Idle();
}


bool calibrado = false; // Indica que o volante está calibrado
bool isCalibrating = false; // Indica que o volante está se deslocando para a origem
unsigned long distance = 0;
unsigned long stopPoint = 0;

// Verifica o estado do volante
void verifyPosition() {
  if (!isCalibrating && last_sw != absolute_sw) { // Entrou ou saiu da chave
    Idle(); // Começa a parar o volante
    last_sw = absolute_sw;
    isCalibrating = true;
    count = 0;
   if (!last_sw && absolute_sw) { // Entrou na chave
     distance = IN_DISTANCE;
     stopPoint = IN_STOP;
   } else if (last_sw && !absolute_sw) { // Saiu da chave
     distance = OUT_DISTANCE;
     stopPoint = OUT_STOP;
   }
  }
}

void calibrate() {
  if (isCalibrating) {
    // const unsigned long delta = distance - delta;
    if (count == distance) {
      Move(0); // move cw; // Manda o volante parar
      isCalibrating = false; // Indica que o processo de calibração se encerrou
      calibrado = true;
    } else if (count > stopPoint) {
      // Se passar ligar o volante no sentido contrário
      // Isso diminui a inércia      
      Move(SPEED, false); // Joga o volante pro outro lado;
    } else {
      // Caso na volta não pare no ponto certo
      // Isso diminui a inércia      
      Move(SPEED); // Joga o volante pro outro lado;
    }
  }
}

void acelera() {
  if (!calibrado) {
    Move(SPEED); // move cw
  } 
}

int dir = ATUA_CCW;
unsigned long mymillis = 0;
unsigned char lastGP = GP_BUTTON;
unsigned long lastClickAt = 0;

void loop() {
  char readGP = PINB & GPB;

  acelera();
   verifyPosition();
   calibrate();

  #if LOG
  // debug only info
    if (millis()%300==0) {
      Serial.print(count);
      Serial.print(", ");
      Serial.println(absolute_sw==true?'1':'0'); 
    }
  #endif
}

ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result == DIR_NONE) {
    // do nothing
  }
  else if (result == DIR_CW) { // Sentido anti-horário
    count--;
  }
  else if (result == DIR_CCW) { // Sentido horário
    count++;
  }

  absolute_sw = 0==(PINB&(1<<POS_SENSOR)); // true = dentro da chave | false = fora da chave
}
