/*
  Torre Sentinela - Arduino UNO

  O PIR e o ultrassonico ficam na parte que gira (servo). Ao armar, a torre
  mapeia o ambiente e depois so trava no que estiver mais perto que o mapa e
  que tenha se mexido. Tudo e enviado ao Raspberry Pi pela USB (115200).

  Mensagens enviadas: OI, EST, R, SUSP, ALVO, TRK, CONF, NIVEL, PERDIDO,
  IGNORADO, MAPA, INSTAVEL, PIR, BASE, BASE_OK, CFG, PONG, MSG
  Comandos recebidos: ARMAR, DESARMAR, CALIBRAR, PING, STATUS, MODO, MIRA,
  LASER, ALCANCE, PATRULHA, SOM, VARRE, MARGEM
*/

#define RGB_ANODO_COMUM 0   // 1 se o LED RGB for de anodo comum
const char VERSAO[] = "3.1";

#include <Servo.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

// Pinos
const uint8_t PIN_PIR          = 2;
const uint8_t PIN_BUZZER       = 3;
const uint8_t LCD_D4 = 4, LCD_D5 = 5, LCD_D6 = 6, LCD_D7 = 7;
const uint8_t LCD_RS           = 8;
const uint8_t PIN_LED_VERDE    = 9;
const uint8_t PIN_BOTAO        = 10;   // botao opcional entre D10 e GND
const uint8_t LCD_E            = A0;
const uint8_t PIN_TRIG         = A1;
const uint8_t PIN_ECHO         = A2;
const uint8_t PIN_SERVO        = A3;
const uint8_t PIN_LASER        = A4;
const uint8_t PIN_LED_VERMELHO = A5;

// Tempos
const int PASSO = 2;                              // graus por passo de varredura
const unsigned long T_PASSO          = 60;        // ms entre passos da varredura
const unsigned long T_RASTREIO       = 80;        // ms entre leituras com alvo travado
const unsigned long T_MANUAL         = 120;
const unsigned long T_LEITURA_SETOR  = 250;       // ms entre leituras parado num setor
const unsigned long T_SETOR          = 9000;      // tempo parado em cada setor ao vigiar
const unsigned long T_PIR_ASSENTAR   = 4000;      // PIR só vale com a cabeça parada há 4 s
const unsigned long T_SEM_MOVIMENTO  = 15000;     // varrendo sem nada -> volta a vigiar
const unsigned long T_ARMANDO        = 5000;      // tempo para sair da área
const unsigned long T_AQUECER_PIR    = 30000;
const unsigned long T_TRK            = 250;
const unsigned long T_SOLTAR_SERVO   = 700;       // em repouso o servo é solto (para de zumbir)
const unsigned long T_SEM_SERVIDOR   = 20000;
const unsigned long T_ABSORVER       = 15000;     // alvo que já se mexeu, parado sem PIR por 15 s -> mapa
const unsigned long T_ABSORVER_SUSP  = 8000;      // algo novo que nunca se mexeu: 8 s parado -> mapa, sem alarme
const uint8_t N_BUSCA = 7;
const uint8_t PASSADAS_FIXO = 3;                  // igual por 3 passadas = faz parte do ambiente

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
Servo servo;

enum Estado : uint8_t { DESARMADO, ARMANDO, CALIBRANDO, VIGIANDO, VARRENDO, TRAVADO, MANUAL };
const char* const NOMES[] = {"DESARMADO", "ARMANDO", "CALIBRANDO", "VIGIANDO", "VARRENDO", "TRAVADO", "MANUAL"};

// Configuração (salva na EEPROM)
struct Config {
  uint8_t magic;
  int16_t alcance;
  uint8_t patrulha;
  uint8_t som;
  uint8_t angMin;
  uint8_t angMax;
  uint8_t margem;     // cm mínimos mais perto que o mapa para contar como alvo
};
const uint8_t CFG_MAGIC = 0x5C;
Config cfg = {CFG_MAGIC, 100, 0, 1, 15, 165, 20};

void salvarCfg() { EEPROM.put(0, cfg); }
void carregarCfg() {
  Config c;
  EEPROM.get(0, c);
  if (c.magic == CFG_MAGIC && c.alcance >= 10 && c.alcance <= 300 &&
      c.angMax <= 180 && c.angMin + 10 <= c.angMax && c.margem >= 5 && c.margem <= 80) cfg = c;
}

// Mapa do ambiente
// Um "balde" a cada 2 graus. Distâncias guardadas em unidades de 2 cm.
uint8_t fundo[91];    // distância normal (0 = sem eco/sem referência)
uint8_t ruido[91];    // quanto a leitura oscila ali (paredes em ângulo oscilam muito)
uint8_t ultima[91];   // o que se viu na última passada
uint8_t estat[91];    // há quantas passadas a leitura não muda
bool calibrado = false;
uint8_t calPasse = 0;
bool calVoltarDesarmado = false;
unsigned long tLiberado = 0;

// Estado
Estado estado = DESARMADO;
bool armadoAntesManual = false;

int  angulo = 90, direcao = 1, vMin = 15, vMax = 165;
uint8_t reversoesJanela = 0, setorIdx = 1;
int  candD = -1;                      // primeira leitura de um possível alvo (espera a confirmação)
uint8_t candN = 0;                    // quantas leituras seguintes bateram com ela
int  alvoAng = 0, alvoDist = 0;
int8_t dirAlvo = 1;
float velAlvo = 0;
uint8_t falhas = 0, idxBusca = 0, nivel = 0;
bool confirmado = false;              // PIR viu movimento no alvo
bool suspeito = false;                // travou, mas ainda não provou que se mexe (sem alarme)
int  angIni = 0, dIni = 0;
int  miraManual = 90;
bool laserManual = false;
int  ultimaDist = -1;
bool pirAtivo = false;
bool pirLimpo = false;   // PIR voltou a zero depois que a cabeça parou

unsigned long tEstado = 0, tPasso = 0, tUltimoMovimento = 0, tBip = 0, tLcd = 0;
unsigned long tAlvo = 0, tTrk = 0, tServoMov = 0, tNivelBaixo = 0, tContato = 0, tSetor = 0;
int refDist = 0, refAng = 0, dVel = 0, dSalto = -1;
uint8_t desvios = 0;
unsigned long tEstavel = 0, tVel = 0, tTravou = 0, tInstavel = 0;
int baldeInstavel = -1;
unsigned int acertos = 0, erros = 0;   // leituras boas e perdidas durante o travamento
bool contatoJaHouve = false, tomAlto = false;

char msgL1[17] = "", msgL2[17] = "";
unsigned long tMsgAte = 0;
char telaL1[17] = "", telaL2[17] = "";

char bufSerial[48];
uint8_t nBuf = 0;

// Saídas
int repouso() { return (cfg.angMin + cfg.angMax) / 2; }
int balde(int ang) { return constrain(ang, 0, 180) / 2; }
uint8_t meio(int d) { return d > 0 ? min(d, 510) / 2 : 0; }   // cm -> unidades de 2 cm

void setRGB(bool vermelho, bool verde) {
#if RGB_ANODO_COMUM
  vermelho = !vermelho; verde = !verde;
#endif
  digitalWrite(PIN_LED_VERMELHO, vermelho);
  digitalWrite(PIN_LED_VERDE, verde);
}

void laser(bool ligado) { digitalWrite(PIN_LASER, ligado ? HIGH : LOW); }

void bip(unsigned int freq, unsigned long dur) { if (cfg.som) tone(PIN_BUZZER, freq, dur); }
void bipOk()   { if (!cfg.som) return; tone(PIN_BUZZER, 1800, 80); delay(110); tone(PIN_BUZZER, 2600, 120); }

void moverServo(int ang) {
  if (!servo.attached()) servo.attach(PIN_SERVO);
  if (ang != servo.read()) { tServoMov = millis(); pirLimpo = false; }   // só conta se mudar de ângulo
  servo.write(ang);
}

void soltarServoParado() {
  if ((estado == DESARMADO || estado == VIGIANDO) && servo.attached() && millis() - tServoMov > T_SOLTAR_SERVO)
    servo.detach();
}

// o PIR gira junto com a torre, entao so vale com ela parada
bool pirConfiavel() {
  unsigned long parado = millis() - tServoMov;
  return (pirLimpo && parado >= T_PIR_ASSENTAR) || parado >= 7000;
}

void mensagem(const char* l1, const char* l2, unsigned long dur) {
  strncpy(msgL1, l1, 16); msgL1[16] = 0;
  strncpy(msgL2, l2, 16); msgL2[16] = 0;
  tMsgAte = millis() + dur;
  tLcd = 0;
}

int ramLivre() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

// Serial
bool estaArmado() { return (estado == MANUAL) ? armadoAntesManual : (estado != DESARMADO && !(estado == CALIBRANDO && calVoltarDesarmado)); }

void enviarEstado() {
  Serial.print(F("EST,")); Serial.print(NOMES[estado]);
  Serial.print(','); Serial.println(estaArmado() ? 1 : 0);
}

void enviarCfg() {
  Serial.print(F("CFG,")); Serial.print(cfg.alcance);
  Serial.print(','); Serial.print(cfg.patrulha);
  Serial.print(','); Serial.print(cfg.som);
  Serial.print(','); Serial.print(cfg.angMin);
  Serial.print(','); Serial.print(cfg.angMax);
  Serial.print(','); Serial.println(cfg.margem);
}

void enviarLeitura(int ang, int d) {
  Serial.print(F("R,")); Serial.print(ang); Serial.print(','); Serial.println(d);
}

// Sensor
int medirCm() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long t = pulseIn(PIN_ECHO, HIGH, 25000UL);  // até ~4 m
  if (t == 0) return -1;
  int cm = t / 58;
  return (cm < 3 || cm > 400) ? -1 : cm;
}

// mais perto que o mapa (olhando os vizinhos, o feixe e largo)
bool ehIntruso(int ang, int d, int alc) {
  if (d <= 0 || d > alc) return false;
  if (!calibrado) return true;
  int i = balde(ang);
  uint8_t ref = 0, rui = 0;
  for (int k = i - 1; k <= i + 1; k++) {
    if (k < 0 || k > 90) continue;
    if (fundo[k] && (ref == 0 || fundo[k] < ref)) ref = fundo[k];
    if (ruido[k] > rui) rui = ruido[k];
  }
  if (ref == 0) return true;
  int bc = ref * 2;
  int margem = max((int)cfg.margem, max(rui * 2, bc / 8));
  return d < bc - margem;
}

// mesma leitura das ultimas passadas = coisa parada
bool ehEstatico(int ang, int d) {
  int i = balde(ang);
  uint8_t v = meio(d);
  return v && ultima[i] && estat[i] >= 2 && abs((int)v - (int)ultima[i]) <= 3;
}

// o que fica igual por algumas passadas vira parte do mapa
void aprender(int ang, int d) {
  int i = balde(ang);
  uint8_t v = meio(d);
  if (v && ultima[i] && abs((int)v - (int)ultima[i]) <= 3) { if (estat[i] < 250) estat[i]++; }
  else estat[i] = 0;
  ultima[i] = v;
  if (calibrado && v && estat[i] >= PASSADAS_FIXO && abs((int)v - (int)fundo[i]) > max(3, (int)ruido[i])) {
    fundo[i] = v;
    estat[i] = 0;
    Serial.print(F("MAPA,")); Serial.print(i * 2); Serial.print(','); Serial.println(v * 2);
  }
}

// Nível de alarme
void definirNivel(uint8_t n) {
  if (n == nivel) return;
  nivel = n;
  Serial.print(F("NIVEL,")); Serial.println(n);
  tLcd = 0;
}

uint8_t calcNivel() {
  if (alvoDist <= cfg.alcance / 2) return 2;
  if (velAlvo < -25 && alvoDist < cfg.alcance * 3 / 4) return 2;
  if (confirmado && alvoDist < cfg.alcance * 3 / 4) return 2;
  return 1;
}

void atualizarNivel() {
  if (estado != TRAVADO) return;
  uint8_t n = calcNivel();
  if (n < nivel) {
    if (tNivelBaixo == 0) tNivelBaixo = millis();
    if (millis() - tNivelBaixo < 2000) return;
  }
  tNivelBaixo = 0;
  definirNivel(n);
}

// Máquina de estados
void mudarEstado(Estado novo) {
  estado = novo;
  tEstado = millis();
  candD = -1;
  if (novo != TRAVADO && novo != MANUAL) laser(false);
  if (novo == DESARMADO || novo == ARMANDO || novo == VIGIANDO) {
    angulo = repouso();
    moverServo(angulo);
  }
  if (novo == VIGIANDO) { setorIdx = 1; tSetor = millis(); }
  if (novo == DESARMADO) noTone(PIN_BUZZER);
  if (novo == VARRENDO) tUltimoMovimento = millis();
  if (novo != TRAVADO) { definirNivel(0); tNivelBaixo = 0; }
  tLcd = 0;
  enviarEstado();
}

void iniciarVarredura(int centro, bool janela) {
  if (janela) {
    vMin = max((int)cfg.angMin, centro - 30);
    vMax = min((int)cfg.angMax, centro + 30);
    reversoesJanela = 3;
  } else {
    vMin = cfg.angMin; vMax = cfg.angMax; reversoesJanela = 0;
  }
  angulo = constrain(centro, vMin, vMax);
  moverServo(angulo);
}

void comecarVarrer(bool janela) {
  int centro = angulo;
  mudarEstado(VARRENDO);
  iniciarVarredura(centro, janela);
}

void iniciarCalibracao(bool voltarDesarmado) {
  calVoltarDesarmado = voltarDesarmado;
  calPasse = 0;
  memset(fundo, 0, sizeof(fundo));   // na calibração: menor leitura
  memset(ruido, 0, sizeof(ruido));   // na calibração: maior leitura
  memset(estat, 0, sizeof(estat));   // na calibração: quantas leituras válidas
  calibrado = false;
  laser(false);
  mudarEstado(CALIBRANDO);
  angulo = cfg.angMin; direcao = 1;
  moverServo(angulo);
  tLiberado = millis() + 500;
}

void terminarCalibracao() {
  int validos = 0;
  for (int k = 0; k <= 90; k++) {
    if (estat[k] == 0) { fundo[k] = 0; ruido[k] = 0; }
    else {
      // leitura isolada muito abaixo das outras = eco falso
      if (estat[k] >= 2 && ultima[k] != 255 && ultima[k] - fundo[k] > 7) fundo[k] = ultima[k];
      ruido[k] = min(ruido[k] - fundo[k], 30);
    }
    ultima[k] = fundo[k];
    estat[k] = 0;
  }
  // angulo bem mais perto que os dois vizinhos = eco falso
  for (int k = 1; k < 90; k++) {
    uint8_t viz = min(fundo[k - 1], fundo[k + 1]);
    if (fundo[k] && viz && fundo[k] + 20 < viz) fundo[k] = viz;
  }
  calibrado = true;
  for (int a = cfg.angMin & ~1; a <= cfg.angMax; a += 2) {
    uint8_t f = fundo[a / 2];
    Serial.print(F("BASE,")); Serial.print(a); Serial.print(',');
    Serial.println(f ? f * 2 : -1);
    if (f) validos++;
  }
  Serial.print(F("BASE_OK,")); Serial.println(validos);
  bip(2400, 200);
  if (calVoltarDesarmado) mudarEstado(DESARMADO);
  else if (cfg.patrulha) comecarVarrer(false);
  else mudarEstado(VIGIANDO);
}

// calibracao: 3 passadas guardando a menor e a maior leitura de cada angulo
void passoCalibracao() {
  int d = medirCm();
  enviarLeitura(angulo, d);
  int i = balde(angulo);
  if (d > 0) {
    // fundo = menor, ultima = 2a menor, ruido = maior
    uint8_t v = meio(d);
    if (estat[i] == 0) { fundo[i] = v; ultima[i] = 255; ruido[i] = v; }
    else {
      if (v < fundo[i]) { ultima[i] = fundo[i]; fundo[i] = v; }
      else if (v < ultima[i]) ultima[i] = v;
      if (v > ruido[i]) ruido[i] = v;
    }
    if (estat[i] < 250) estat[i]++;
  }
  angulo += direcao * PASSO;
  if (angulo > cfg.angMax) {
    if (calPasse >= 2) { terminarCalibracao(); return; }
    angulo = cfg.angMax; direcao = -1; calPasse++;
  } else if (angulo < cfg.angMin) {
    angulo = cfg.angMin; direcao = 1; calPasse++;
  }
  moverServo(angulo);
}

int progressoCalibracao() {
  long faixa = max(1, cfg.angMax - cfg.angMin);
  long pos = (direcao > 0) ? angulo - cfg.angMin : cfg.angMax - angulo;
  return constrain((calPasse * faixa + pos) * 100 / (3 * faixa), 0, 99);
}

void armar() {
  if (estado == DESARMADO || estado == MANUAL || (estado == CALIBRANDO && calVoltarDesarmado)) {
    laserManual = false;
    mudarEstado(ARMANDO);
  }
}

void desarmar() {
  laserManual = false;
  mudarEstado(DESARMADO);
}

void travar(int ang, int d) {
  alvoAng = ang; alvoDist = d; falhas = 0; idxBusca = 0;
  velAlvo = 0; tAlvo = millis(); dirAlvo = direcao;
  tUltimoMovimento = millis();
  refDist = d; refAng = ang; tEstavel = millis(); desvios = 0;
  dVel = d; tVel = millis(); dSalto = -1; tTravou = millis(); acertos = erros = 0;
  confirmado = false;
  suspeito = true; angIni = ang; dIni = d;
  mudarEstado(TRAVADO);
  laser(true);
  Serial.print(F("SUSP,")); Serial.print(ang); Serial.print(','); Serial.println(d);
}

// alvo se mexeu (ou o PIR viu): vira alarme
void confirmarAlvo() {
  if (!suspeito) return;
  suspeito = false;
  Serial.print(F("ALVO,")); Serial.print(alvoAng); Serial.print(','); Serial.println(alvoDist);
  definirNivel(calcNivel());
  bip(3000, 150);
  tLcd = 0;
}

// travou e perdeu rapido duas vezes no mesmo lugar: reflexo, ignora ali
void marcarInstavel() {
  int b = balde(alvoAng);
  if (baldeInstavel >= 0 && abs(b - baldeInstavel) <= 3 && millis() - tInstavel < 180000UL) {
    uint8_t v = meio(alvoDist);
    for (int k = min(b, baldeInstavel) - 3; k <= max(b, baldeInstavel) + 3; k++) {
      if (k < 0 || k > 90 || !fundo[k] || fundo[k] <= v) continue;
      int precisa = fundo[k] - v + 3;
      if (precisa > ruido[k]) ruido[k] = min(precisa, 120);
    }
    Serial.print(F("INSTAVEL,")); Serial.print(alvoAng); Serial.print(','); Serial.println(alvoDist);
    baldeInstavel = -1;
  } else {
    baldeInstavel = b;
    tInstavel = millis();
  }
}

void perdeuAlvo() {
  bool piscando = (unsigned long)erros * 10 > (unsigned long)(acertos + erros) * 3;
  if (!confirmado && (millis() - tTravou < 4000 || piscando)) marcarInstavel();
  Serial.println(F("PERDIDO"));
  mudarEstado(VARRENDO);
  direcao = dirAlvo;
  iniciarVarredura(alvoAng, true);
}

// objeto parado que nao se mexeu: entra no mapa
void absorverAlvo() {
  uint8_t v = meio(alvoDist + 2);
  int i = balde(alvoAng);
  for (int k = i - 4; k <= i + 4; k++) {
    if (k < 0 || k > 90) continue;
    if (fundo[k] == 0 || fundo[k] > v) fundo[k] = v;
    if (ruido[k] < 3) ruido[k] = 3;
    ultima[k] = v;
    estat[k] = PASSADAS_FIXO;
  }
  calibrado = true;
  Serial.print(F("IGNORADO,")); Serial.print(alvoAng); Serial.print(','); Serial.println(alvoDist);
  mensagem("Objeto parado", "add ao mapa", 2000);
  mudarEstado(VARRENDO);
  iniciarVarredura(alvoAng, false);
}

void fimDeCurso() {
  if (reversoesJanela > 0 && --reversoesJanela == 0) { vMin = cfg.angMin; vMax = cfg.angMax; }
}

// precisa de 3 leituras parecidas para travar
bool avaliar(int ang, int d) {
  bool candidato = ehIntruso(ang, d, cfg.alcance) && !ehEstatico(ang, d);
  if (candD > 0) {
    if (candidato && abs(d - candD) <= 12) {
      if (++candN >= 2) { candD = -1; candN = 0; travar(ang, d); return true; }
      return false;
    }
    candD = -1; candN = 0;
    return false;
  }
  if (candidato) { candD = d; candN = 0; }
  return false;
}

void passoVarredura() {
  int d = medirCm();
  ultimaDist = d;
  enviarLeitura(angulo, d);
  if (avaliar(angulo, d)) return;
  if (candD > 0) return;
  aprender(angulo, d);
  angulo += direcao * PASSO;
  if (angulo >= vMax) { angulo = vMax; direcao = -1; fimDeCurso(); }
  else if (angulo <= vMin) { angulo = vMin; direcao = 1; fimDeCurso(); }
  moverServo(angulo);
}

int offsetBusca(uint8_t k) {
  if (k == 0) return 0;
  int mag = ((k + 1) / 2) * 4;
  return ((k % 2 == 1) ? dirAlvo : -dirAlvo) * mag;
}

void passoRastreio() {
  int d = medirCm();
  ultimaDist = d;
  enviarLeitura(angulo, d);
  // pulo grande so vale se a proxima leitura confirmar
  bool salto = false;
  if (d > 0 && abs(d - alvoDist) > 30) {
    if (dSalto > 0 && abs(d - dSalto) <= 12) dSalto = -1;
    else { dSalto = d; salto = true; }
  } else dSalto = -1;
  if (ehIntruso(angulo, d, cfg.alcance + 15) && !salto) {
    unsigned long agora = millis();
    if (agora - tVel >= 500) {
      float inst = constrain((d - dVel) * 1000.0 / (agora - tVel), -300.0, 300.0);
      velAlvo = velAlvo * 0.5 + inst * 0.5;
      dVel = d; tVel = agora;
    }
    if (angulo != alvoAng) dirAlvo = (angulo > alvoAng) ? 1 : -1;
    alvoAng = angulo; alvoDist = d; tAlvo = agora;
    falhas = 0; idxBusca = 0;
    if (acertos < 60000) acertos++;
    tUltimoMovimento = agora;
    if (suspeito && (abs(d - dIni) >= 20 || abs(angulo - angIni) >= 14 || abs(velAlvo) >= 20)) confirmarAlvo();
    if (abs(d - refDist) > 8 || abs(angulo - refAng) > 6) {
      if (++desvios >= 3) { refDist = d; refAng = angulo; tEstavel = agora; desvios = 0; }
    } else if (agora - tEstavel > (suspeito ? T_ABSORVER_SUSP : T_ABSORVER) && !confirmado && !pirAtivo
               && agora - tServoMov > T_PIR_ASSENTAR + 2000
              ) {
      absorverAlvo();
      return;
    }
    if (agora - tTrk >= T_TRK) {
      tTrk = agora;
      Serial.print(F("TRK,")); Serial.print(alvoAng); Serial.print(',');
      Serial.print(alvoDist); Serial.print(','); Serial.println((int)velAlvo);
    }
    return;
  }
  if (erros < 60000) erros++;
  if (++falhas < 2) return;
  idxBusca++;
  if (idxBusca >= N_BUSCA) { perdeuAlvo(); return; }
  angulo = constrain(alvoAng + offsetBusca(idxBusca), cfg.angMin, cfg.angMax);
  moverServo(angulo);
}

void somAlarme(unsigned long agora) {
  if (!cfg.som) return;
  if (nivel >= 2) {
    if (agora - tBip >= 180) { tBip = agora; tomAlto = !tomAlto; tone(PIN_BUZZER, tomAlto ? 2500 : 1700, 170); }
  } else {
    long intervalo = map(constrain(alvoDist, 5, cfg.alcance), 5, cfg.alcance, 90, 700);
    if (agora - tBip >= (unsigned long)intervalo) { tBip = agora; tone(PIN_BUZZER, 2200, 40); }
  }
}

int anguloSetor(uint8_t k) {   // 3 setores: 1/6, 3/6 e 5/6 da área de varredura
  return cfg.angMin + (long)(cfg.angMax - cfg.angMin) * (1 + 2 * k) / 6;
}

void vigiar(unsigned long agora) {
  if (cfg.patrulha) { comecarVarrer(false); return; }
  if (pirAtivo) { comecarVarrer(true); return; }   // procura perto de onde estava olhando
  if (agora - tSetor >= T_SETOR) {
    setorIdx = (setorIdx + 1) % 3;
    angulo = anguloSetor(setorIdx);
    moverServo(angulo);
    tSetor = agora;
    candD = -1;
  }
  if (agora - tServoMov > 400 && agora - tPasso >= T_LEITURA_SETOR) {
    tPasso = agora;
    int d = medirCm();
    ultimaDist = d;
    enviarLeitura(angulo, d);
    avaliar(angulo, d);
  }
}

void atualizarMaquina() {
  unsigned long agora = millis();
  switch (estado) {
    case DESARMADO:
      break;

    case ARMANDO:
      if (agora - tBip >= 1000) { tBip = agora; bip(1500, 60); }
      if (agora - tEstado >= T_ARMANDO) iniciarCalibracao(false);
      break;

    case CALIBRANDO:
      if ((long)(agora - tLiberado) < 0) break;
      if (agora - tPasso >= T_PASSO) { tPasso = agora; passoCalibracao(); }
      break;

    case VIGIANDO:
      vigiar(agora);
      break;

    case VARRENDO:
      if (pirAtivo) tUltimoMovimento = agora;
      if (!cfg.patrulha && agora - tUltimoMovimento > T_SEM_MOVIMENTO) { mudarEstado(VIGIANDO); break; }
      if (agora - tPasso >= T_PASSO) { tPasso = agora; passoVarredura(); }
      break;

    case TRAVADO:
      if (pirAtivo && !confirmado) {
        confirmado = true;
        Serial.println(F("CONF"));
        confirmarAlvo();
        tLcd = 0;
      }
      if (agora - tPasso >= T_RASTREIO) { tPasso = agora; passoRastreio(); }
      if (estado != TRAVADO || suspeito) break;
      atualizarNivel();
      somAlarme(agora);
      break;

    case MANUAL:
      if (agora - tPasso >= T_MANUAL) {
        tPasso = agora;
        ultimaDist = medirCm();
        enviarLeitura(miraManual, ultimaDist);
      }
      break;
  }
  soltarServoParado();
}

// PIR
void lerPIR() {
  if (millis() < T_AQUECER_PIR) return;
  bool bruto = digitalRead(PIN_PIR) == HIGH;
  if (!bruto && millis() - tServoMov >= 1000) pirLimpo = true;
  bool agora = bruto && pirConfiavel();
  if (agora != pirAtivo) {
    pirAtivo = agora;
    Serial.print(F("PIR,")); Serial.println(agora ? 1 : 0);
    tLcd = 0;
  }
}

// Botao local (opcional)
void lerBotao() {
  static bool anterior = HIGH;
  static unsigned long tMudou = 0;
  bool agora = digitalRead(PIN_BOTAO);
  if (agora != anterior && millis() - tMudou > 50) {
    tMudou = millis();
    anterior = agora;
    if (agora == LOW) {
      Serial.println(F("MSG,botao pressionado"));
      if (estaArmado()) { desarmar(); mensagem("Botao", "Desarmado", 2000); }
      else { armar(); mensagem("Botao", "Armando...", 2000); }
      bipOk();
    }
  }
}

// Comandos do Raspberry
void processarComando(char* linha) {
  tContato = millis();
  contatoJaHouve = true;
  char* arg = strchr(linha, ',');
  if (arg) *arg++ = 0;
  int valor = arg ? atoi(arg) : 0;

  if (!strcmp(linha, "PING")) {
    Serial.print(F("PONG,")); Serial.print(millis() / 1000);
    Serial.print(','); Serial.print(ramLivre());
    Serial.print(','); Serial.println(calibrado ? 1 : 0);
  }
  else if (!strcmp(linha, "ARMAR")) armar();
  else if (!strcmp(linha, "DESARMAR")) desarmar();
  else if (!strcmp(linha, "CALIBRAR")) {
    if (estado == DESARMADO) iniciarCalibracao(true);
    else if (estado == VIGIANDO || estado == VARRENDO) iniciarCalibracao(false);
    else Serial.println(F("MSG,calibracao so com a torre desarmada ou vigiando"));
  }
  else if (!strcmp(linha, "MODO") && arg) {
    if (!strcmp(arg, "MANUAL") && estado != MANUAL) {
      armadoAntesManual = estaArmado();
      miraManual = angulo;
      laserManual = false;
      laser(false);
      mudarEstado(MANUAL);
    } else if (!strcmp(arg, "AUTO") && estado == MANUAL) {
      laserManual = false;
      mudarEstado(armadoAntesManual ? VIGIANDO : DESARMADO);
    }
  }
  else if (!strcmp(linha, "MIRA")) {
    if (estado == MANUAL) { miraManual = constrain(valor, 0, 180); moverServo(miraManual); tLcd = 0; }
  }
  else if (!strcmp(linha, "LASER")) {
    if (estado == MANUAL) { laserManual = valor != 0; laser(laserManual); tLcd = 0; }
  }
  else if (!strcmp(linha, "ALCANCE")) { cfg.alcance = constrain(valor, 10, 300); salvarCfg(); enviarCfg(); tLcd = 0; }
  else if (!strcmp(linha, "PATRULHA")) { cfg.patrulha = valor != 0; salvarCfg(); enviarCfg(); }
  else if (!strcmp(linha, "SOM")) { cfg.som = valor != 0; if (!cfg.som) noTone(PIN_BUZZER); salvarCfg(); enviarCfg(); }
  else if (!strcmp(linha, "MARGEM")) { cfg.margem = constrain(valor, 5, 80); salvarCfg(); enviarCfg(); }
  else if (!strcmp(linha, "VARRE") && arg) {
    char* arg2 = strchr(arg, ',');
    int mn = constrain(valor, 0, 170);
    int mx = arg2 ? constrain(atoi(arg2 + 1), mn + 10, 180) : cfg.angMax;
    cfg.angMin = mn; cfg.angMax = mx;
    salvarCfg(); enviarCfg();
    if (estado == CALIBRANDO) iniciarCalibracao(calVoltarDesarmado);
    else if (estado == VARRENDO) iniciarVarredura(constrain(angulo, mn, mx), false);
    else if (estado == DESARMADO || estado == VIGIANDO || estado == ARMANDO) { angulo = repouso(); moverServo(angulo); }
  }
  else if (!strcmp(linha, "STATUS")) { enviarEstado(); enviarCfg(); }
  else { Serial.print(F("MSG,comando desconhecido: ")); Serial.println(linha); }
}

void lerSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      bufSerial[nBuf] = 0;
      if (nBuf > 0) processarComando(bufSerial);
      nBuf = 0;
    } else if (nBuf < sizeof(bufSerial) - 1) {
      bufSerial[nBuf++] = c;
    } else {
      nBuf = 0;
    }
  }
}

// LED e LCD
void atualizarLed() {
  unsigned long t = millis();
  switch (estado) {
    case DESARMADO:  setRGB(false, true); break;
    case ARMANDO:    setRGB((t / 250) % 2, (t / 250) % 2); break;
    case CALIBRANDO: setRGB((t / 100) % 2, (t / 100) % 2); break;
    case VIGIANDO:   setRGB(true, true); break;
    case VARRENDO:   setRGB(true, (t / 400) % 2); break;
    case TRAVADO:
      if (suspeito) setRGB((t / 120) % 2, (t / 120) % 2);
      else setRGB((t / (nivel >= 2 ? 70 : 150)) % 2, false);
      break;
    case MANUAL:     setRGB(false, (t / 600) % 2); break;
  }
}

void escreverLinha(uint8_t linha, const char* texto, char* anterior) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", texto);
  if (strcmp(buf, anterior) == 0) return;
  strcpy(anterior, buf);
  lcd.setCursor(0, linha);
  lcd.print(buf);
}

void atualizarLcd() {
  if (millis() - tLcd < 250) return;
  tLcd = millis();
  char l1[17], l2[17], dist[6];
  if (ultimaDist > 0) snprintf(dist, sizeof(dist), "%dcm", ultimaDist);
  else strcpy(dist, "---");

  if (millis() < tMsgAte) { escreverLinha(0, msgL1, telaL1); escreverLinha(1, msgL2, telaL2); return; }

  switch (estado) {
    case DESARMADO:
      strcpy(l1, "DESARMADO");
      if (millis() < T_AQUECER_PIR) strcpy(l2, "Aquecendo PIR...");
      else if (contatoJaHouve && millis() - tContato > T_SEM_SERVIDOR) strcpy(l2, "Sem servidor");
      else strcpy(l2, "Arme pelo painel");
      break;
    case ARMANDO:
      snprintf(l1, sizeof(l1), "ARMANDO em %lus", (T_ARMANDO - (millis() - tEstado)) / 1000 + 1);
      strcpy(l2, "Saia da area!");
      break;
    case CALIBRANDO:
      snprintf(l1, sizeof(l1), "CALIBRANDO %3d%%", progressoCalibracao());
      strcpy(l2, "Mapeando area...");
      break;
    case VIGIANDO:
      snprintf(l1, sizeof(l1), "VIGIANDO  PIR:%s", pirAtivo ? "!" : (pirConfiavel() ? "-" : "~"));
      snprintf(l2, sizeof(l2), "Setor %d/3 %s", setorIdx + 1, dist);
      break;
    case VARRENDO:
      snprintf(l1, sizeof(l1), "VARRENDO   %3d%c", angulo, (char)223);
      snprintf(l2, sizeof(l2), "Dist: %s", dist);
      break;
    case TRAVADO: {
      strcpy(l1, suspeito ? "VERIFICANDO..." : (nivel >= 2 ? "!!! ALERTA !!!" : (confirmado ? "ALVO CONFIRMADO" : "ALVO TRAVADO")));
      const char* v = velAlvo < -10 ? "<<" : (velAlvo > 10 ? ">>" : "--");
      snprintf(l2, sizeof(l2), "%3dcm %3d%c %s", alvoDist, alvoAng, (char)223, v);
      break;
    }
    case MANUAL:
      snprintf(l1, sizeof(l1), "MANUAL Laser:%s", laserManual ? "ON" : "--");
      snprintf(l2, sizeof(l2), "Ang %3d  %s", miraManual, dist);
      break;
  }
  escreverLinha(0, l1, telaL1);
  escreverLinha(1, l2, telaL2);
}

// Setup / loop
void setup() {
  Serial.begin(115200);
  carregarCfg();

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LASER, OUTPUT);
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_VERMELHO, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BOTAO, INPUT_PULLUP);
  laser(false);

  lcd.begin(16, 2);
  lcd.print(F("Torre Sentinela"));
  lcd.setCursor(0, 1);
  lcd.print(F("v")); lcd.print(VERSAO); lcd.print(F(" iniciando"));

  angulo = repouso();
  moverServo(angulo);

  Serial.print(F("OI,TorreSentinela,")); Serial.println(VERSAO);
  bip(1200, 80); delay(120); bip(1800, 80);
  delay(600);
  lcd.clear();
  mudarEstado(DESARMADO);
  enviarCfg();
}

void loop() {
  lerSerial();
  lerBotao();
  lerPIR();
  atualizarMaquina();
  atualizarLed();
  atualizarLcd();
}
