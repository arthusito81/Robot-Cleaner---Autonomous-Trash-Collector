// ---------------------------------------------------------------- //
// Arduino Jeep Bot - Robô Coletor de Lixo
// Anda 5 s, para, gira 5 s no próprio eixo e repete até achar
// um objeto (até 30 cm). Chega perto, e a 15 cm abaixa a pá para coletar.
// A pá fica sempre de pé (levantada) e só abaixa na hora da coleta.
// ---------------------------------------------------------------- //

#include <Servo.h>

// ----------------------------- PINOS ----------------------------- //
int echoPin  = 8;   // D8  -> Echo do HC-SR04 (sensor virado para a FRENTE)
int trigPin  = 7;   // D7  -> Trig do HC-SR04
int in1      = 4;   // Motor A - Pino 1
int in2      = 5;   // Motor A - Pino 2
int in3      = 9;   // Motor B - Pino 1
int in4      = 10;  // Motor B - Pino 2
int servoPin = 6;   // D6  -> Sinal do servo da pá

// ------------------- AJUSTES (calibre no robô) ------------------- //
const int SHOVEL_UP   = 100;  // Ângulo da pá de pé (levantada) - posição padrão
const int SHOVEL_DOWN = 20;   // Ângulo da pá abaixada (encostada no chão)

const int DETECT_DIST = 30;   // cm: enxerga o objeto até aqui -> vai até ele
const int PICKUP_DIST = 15;   // cm: chegou perto -> abaixa a pá e coleta

const unsigned long WANDER_TIME    = 5000; // ms andando reto antes de parar e girar

// ---- MODO POUCA ENERGIA (bateria 9V fraca): tudo mais devagar e mais suave ----
// Com pouca tensão os motores andam/giram menos por milissegundo, então os tempos são maiores.
const unsigned long SPIN_TIME = 5000; // ms (tempo real, relógio) girando no próprio eixo procurando objeto
const int SPIN_STEP_MS   = 200; // ms de cada "passinho" de giro (pulso maior vence o atrito)
const int SPIN_SETTLE_MS = 150; // ms parado depois de cada passinho (tensão se recupera + leitura estável)

const int PUSH_MS = 800;        // ms avançando com a pá abaixada (0 = só abaixar já basta)
const int BACK_MS = 600;        // ms de ré depois de levantar a pá com o lixo

const int STAGGER_MS     = 50;  // ms entre ligar o motor A e o B (evita pico de corrente junto)
const int SHOVEL_STEP_MS = 30;  // ms por grau do servo (15 = rápido, 30 = suave e gasta menos pico)

// ---------------------------- VARIÁVEIS -------------------------- //
Servo shovel;
int shovelAngle = SHOVEL_UP;

long duration;
int distance;

enum State { WANDER, SCAN, APPROACH };
State state = WANDER;
unsigned long stateStart = 0;  // Quando o estado atual começou (ms)
int scanDir = 0;               // 0 = esquerda, 1 = direita

// ------------------------------ SETUP ---------------------------- //
void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  shovel.attach(servoPin);
  shovel.write(SHOVEL_UP);    // A pá começa (e fica) de pé

  randomSeed(analogRead(A0)); // A0 sem nada ligado = ruído = sorteio diferente a cada boot
  stateStart = millis();

  Serial.begin(9600);
}

// ------------------------------ LOOP ----------------------------- //
void loop() {
  distance = readDistance();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= DETECT_DIST) {
    // 1. Viu algo a até 30 cm pela primeira vez: para e confirma a leitura
    if (state != APPROACH) {
      stopMotors();
      delay(SPIN_SETTLE_MS);
      if (readStableDistance() <= DETECT_DIST) {
        state = APPROACH;     // Confirmado: é um objeto
      }
      // Se não confirmou, foi falso alarme: segue a busca normal abaixo
    }

    // 2. Aproximando: anda até chegar a 15 cm
    if (state == APPROACH) {
      if (distance <= PICKUP_DIST) {
        stopMotors();
        delay(SPIN_SETTLE_MS);
        if (readStableDistance() <= PICKUP_DIST) {
          pickUpTrash();      // Chegou: abaixa a pá e coleta
          return;
        }
      }
      forward();
      delay(30);
      return;
    }
  } else if (state == APPROACH) {
    // Perdeu o objeto de vista durante a aproximação: volta a procurar girando
    stopMotors();
    state = SCAN;
    stateStart = millis();   // Começa a contar os 5 s de giro
    scanDir = random(2);
  }

  // 3. Nada à vista: anda 5 s, para e gira 5 s no próprio eixo, e repete
  if (state == WANDER) {
    forward();
    if (millis() - stateStart >= WANDER_TIME) {
      stopMotors();
      delay(SPIN_SETTLE_MS);
      state = SCAN;
      stateStart = millis();  // Começa a contar os 5 s de giro
      scanDir = random(2);    // Sorteia o lado da varredura
    }
    delay(30);
  } else {
    scanStep();
  }
}

// ------------------------ LEITURA DO SENSOR ---------------------- //
int readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Timeout de 25000 us (~4 m) para evitar o bug de 804 cm
  duration = pulseIn(echoPin, HIGH, 25000);

  if (duration == 0) {
    return 999; // Não leu nada = caminho totalmente livre
  }
  return duration * 0.034 / 2;
}

// Lê 3 vezes e devolve o valor do meio (ignora leituras malucas)
int readStableDistance() {
  int a = readDistance();
  delay(30);
  int b = readDistance();
  delay(30);
  int c = readDistance();

  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
}

// ---------------------- GIRO NO PRÓPRIO EIXO --------------------- //
// Um passinho de giro: liga, para e espera assentar.
// (Assim o sensor lê com o robô parado e o giro fica mais previsível.)
void spinStep(int dir) {
  if (dir == 0) {
    turnLeft();
  } else {
    turnRight();
  }
  delay(SPIN_STEP_MS);
  stopMotors();
  delay(SPIN_SETTLE_MS);
}

// Gira em passinhos até completar "ms" de tempo ligado
void spinInPlace(unsigned long ms, int dir) {
  unsigned long spun = 0;
  while (spun < ms) {
    spinStep(dir);
    spun += SPIN_STEP_MS;
  }
}

// Varredura: um passinho por volta do loop(); o loop() lê o sensor entre eles
void scanStep() {
  spinStep(scanDir);

  // Girou por 5 s sem achar nada: volta a andar 5 s
  if (millis() - stateStart >= SPIN_TIME) {
    state = WANDER;
    stateStart = millis();
  }
}

// ------------------------- COLETA DE LIXO ------------------------ //
void pickUpTrash() {
  stopMotors();
  delay(200);

  moveShovel(SHOVEL_DOWN);   // 1. Só agora a pá abaixa (motores parados)

  forward();                 // 2. Avança um pouco para empurrar o objeto para a pá
  delay(PUSH_MS);
  stopMotors();
  delay(200);

  moveShovel(SHOVEL_UP);     // 3. Pá volta a ficar de pé, com o lixo

  backward();                // 4. Dá ré para sair de perto
  delay(BACK_MS);
  stopMotors();
  delay(100);

  // 5. Vira um ângulo aleatório (no eixo) e recomeça o ciclo de busca
  spinInPlace(random(300, 900), random(2));
  state = WANDER;
  stateStart = millis();
}

// Move o servo devagar (1 grau por vez) para não dar tranco nem puxar muita corrente
void moveShovel(int target) {
  while (shovelAngle != target) {
    if (target > shovelAngle) {
      shovelAngle++;
    } else {
      shovelAngle--;
    }
    shovel.write(shovelAngle);
    delay(SHOVEL_STEP_MS);
  }
}

// ----------------------------- MOTORES --------------------------- //
// Se algum movimento sair invertido, troque HIGH/LOW (ou os fios do motor) só aqui.
// Arranque escalonado: liga o motor A, espera um pouco e liga o B, para os dois
// não puxarem o pico de corrente ao mesmo tempo (a bateria fraca agradece).
// O robô sempre PARA antes de mudar de sentido (sem inversão direta = sem pico extra).

int lastA = 0;   // Último comando do motor A: 1 = frente, -1 = trás, 0 = parado
int lastB = 0;   // Último comando do motor B

void motorA(int dir) {
  if (dir == 1)       { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else if (dir == -1) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
}

void motorB(int dir) {
  if (dir == 1)       { digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); }
  else if (dir == -1) { digitalWrite(in3, HIGH); digitalWrite(in4, LOW);  }
  else                { digitalWrite(in3, LOW);  digitalWrite(in4, LOW);  }
}

void drive(int a, int b) {
  if (a == lastA && b == lastB) return;          // Já está assim: não mexe
  motorA(a);
  if (b != 0 && lastB == 0) delay(STAGGER_MS);   // B estava parado: espera antes de ligar
  motorB(b);
  lastA = a;
  lastB = b;
}

void forward()    { drive( 1,  1); }
void backward()   { drive(-1, -1); }
void turnLeft()   { drive(-1,  1); }   // A para trás, B para frente (gira no próprio eixo)
void turnRight()  { drive( 1, -1); }   // A para frente, B para trás (gira no próprio eixo)
void stopMotors() { drive( 0,  0); }