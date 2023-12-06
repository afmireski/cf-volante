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

#define COUNT -770 // Distância da entrada da chave até o ponto de origem
#define PADDING 10

#define STOP false // Controle de debug
#define LOG true
#define SPEED 160


#include "Rotary.h"
volatile long count = 0; // encoder_rotativo = posicao relativa depois de ligado
volatile bool absolute_sw = false; // chave de posicao do volante ativa?
bool last_sw = false;

Rotary r = Rotary(ROTARY_ENC_A, ROTARY_ENC_B);

unsigned long lastVerify = 0; // Usado para debounce do last_sw

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
  acelera();
}


bool calibrado = false; // Indica que o volante está calibrado
bool isCalibrating = false; // Indica que o volante está se deslocando para a origem
unsigned long distance = 0;
unsigned long stepSpeed = 0;


// Verifica o estado do volante
void verifyPosition() {
  if (!isCalibrating && !calibrado && last_sw != absolute_sw && (millis() - lastVerify > 1)) { // Entrou ou saiu da chave
    lastVerify = millis();
    if (last_sw && !absolute_sw) { // Verifica se entrou na chave
      Idle(); // Começa a parar o volante
      isCalibrating = true;
      count = COUNT;
    }
    last_sw = absolute_sw;
  }
}

void calibrate() {
  if (isCalibrating && !calibrado) {
    if (count >= PADDING) {      
      Move(SPEED-stepSpeed, false);
      stepSpeed+=PADDING;
     }
     if (count <= -PADDING) {   
        Move(SPEED-stepSpeed);
        stepSpeed+=PADDING;
     } 
     if (count > -PADDING && count < PADDING) {
       Move(0);
       if (stepSpeed==SPEED) {
         isCalibrating = false; // Indica que o processo de calibração se encerrou
         calibrado = true;           
       }
     }
  }
}

void acelera() {
  if (!calibrado && !isCalibrating) {
    Move(SPEED); // move cw
  } 
}

int dir = ATUA_CCW;
unsigned long mymillis = 0;
unsigned char lastGP = GP_BUTTON;
unsigned long lastClickAt = 0;

void loop() {
  char readGP = PINB & GPB;

  #if STOP
    Move(0); // move cw; // Manda o volante parar
  #else
    verifyPosition();
    calibrate();  
  #endif

  #if LOG
  // debug only info
    if (millis()%300==0) {      
      Serial.print(count);
      Serial.print(", ");
      Serial.print(absolute_sw==true?'1':'0'); 
      Serial.print(", ");
      Serial.println(last_sw==true?'1':'0'); 
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

  absolute_sw = 0==(PINB&(1<<POS_SENSOR)); // false = dentro da chave | true = fora da chave
}
