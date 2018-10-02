// Programa: TCC No-Break

// BIBLIOTECAS
#include <Adafruit_ADS1015.h>  // Conversor AD
#include <Wire.h>              // Protocolo I2C
#include <LiquidCrystal_I2C.h> // LCD I2C
#include <OneWire.h>           // Comunicacao Sensor de Temperatura
#include <DallasTemperature.h> // Sensor de Temperatura


// DEFINICOES I/O
// DIGITAIS
#define BOTAO                  2 // In
#define SENSOR_TEMPERATURA     3 // In
#define RELE_CARREGAMENTOPLACA 4 // Out
#define RELE_CARREGAMENTOREDE  5 // Out
#define RELE_CARGA             6 // Out
#define RELE_EXAUSTOR          7 // Out

// ANALOGICAS
#define SENSORAC_REDE          0
#define SENSORAC_CARGA         1
#define SENSORDC_BATERIA       2
#define SENSORDC_PLACASOLAR    3


// ENDERECOS I2C
#define LCD_ADD 0x3f
#define CONVERSORAD_ADD 0x48

// VARIAVEIS
float tempMin = 999;
float tempMax = 0;
float tempC = 0;


int buttonState = 0; // Ate quatro
boolean current_up = LOW;
boolean last_up=LOW;


// Variaveis para conversao AD
int16_t adc0, adc1, adc2, adc3;
float sensorAcRede, sensorAcCarga, sensorDcBateria, sensorDcPlacaSolar = 0.0;


// INICIALIZACAO
LiquidCrystal_I2C lcd(LCD_ADD,2,1,0,4,5,6,7,3, POSITIVE); // LCD
Adafruit_ADS1115 ads(CONVERSORAD_ADD);                    // Conversor AD
OneWire oneWire(SENSOR_TEMPERATURA);                      // Protocolo de comunicacao sensor de temperatura
DallasTemperature sensors(&oneWire);                      // Sensor de temperatura
DeviceAddress sensor1;                                    // Endereco sensor de temperatura


void setup(void) {
  Serial.begin(9600);

  // Sensor de temperatura
  sensors.begin();

  // Conversor AD
  ads.begin();
  
  // Localiza e mostra enderecos dos sensores
  Serial.println("Localizando sensores DS18B20...");
  Serial.print("Foram encontrados ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" sensores.");
  if (!sensors.getAddress(sensor1, 0))
     Serial.println("Sensores nao encontrados !");
     
  // Mostra o endereco do sensor encontrado no barramento
  Serial.print("Endereco sensor: ");
  mostra_endereco_sensor(sensor1);
  Serial.println();
  Serial.println();
  lcd.begin(16, 2);

  pinMode(BOTAO, INPUT);
  pinMode(RELE_CARREGAMENTOPLACA, OUTPUT);
  pinMode(RELE_CARREGAMENTOREDE, OUTPUT);
  pinMode(RELE_CARGA, OUTPUT);
  pinMode(RELE_EXAUSTOR, OUTPUT);

  // Define o time interrupt para 8khz
  cli();
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2  = 0;
    OCR2A = 249;             // = (16*10^6) / (8000*8) - 1 (deve ser menor < 256)
    TCCR2A |= (1 << WGM21);
    TCCR2B |= (1 << CS21);
    TIMSK2 |= (1 << OCIE2A);
  sei();
}

ISR(TIMER2_COMPA_vect) {
  // Time interrupt
  if (sensorDcPlacaSolar >= 2.5 || sensorAcRede < 0.6) {
    digitalWrite(6, HIGH);
  } else {
    digitalWrite(6, LOW);
  }
}

void mostra_endereco_sensor(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    // Adiciona zeros se necessário
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void carregamentoBateria() {
  // Carregamento da bateria Se tensao placa menor que XXXXX carrega pela rede
  if (sensorDcPlacaSolar >= 2.5) {
    digitalWrite(RELE_CARREGAMENTOPLACA, LOW);
    digitalWrite(RELE_CARREGAMENTOREDE, HIGH);
  } else {
    digitalWrite(RELE_CARREGAMENTOREDE, LOW);
    digitalWrite(RELE_CARREGAMENTOPLACA, HIGH);
  }
}

void lcdBacklight() {
  // Acende e apaga backlight do LCD
  // Acao do botao para Backlight
  lcd.setBacklight(LOW);
  
  // faz a leitura do valor do botao
  buttonState = digitalRead(BOTAO);
  
  // verifica se o botao esta pressionado
  if (buttonState == HIGH) {
    lcd.setBacklight(HIGH);
  } else {
    lcd.setBacklight(LOW);
  }
}

void temperatura() {
  // Le a informacao do sensor
  sensors.requestTemperatures();
  tempC = sensors.getTempC(sensor1);
  // Atualiza temperaturas minima e maxima
  if (tempC < tempMin) {
    tempMin = tempC;
  }
  if (tempC > tempMax) {
    tempMax = tempC;
  }

  // Aciona o exaustor se a temperatura estiver acima de 25 graus celsius
  if (tempC >= 25) {
    // FAN (representado pelo LED) 
    digitalWrite(7, HIGH); 
  } else {
    digitalWrite(7, LOW);
  }

  // Mostra dados no serial monitor
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Min: ");
  Serial.print(tempMin);
  Serial.print(" Max: ");
  Serial.println(tempMax);

  // Mostra dados no LCD  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp.:       ");

  // Simbolo grau
  lcd.write(223);
  lcd.print("C");
  lcd.setCursor(7,0);
  lcd.print(tempC);
  lcd.setCursor(0,1);
  lcd.print("L: ");
  lcd.setCursor(3,1);
  lcd.print(tempMin,1);
  lcd.setCursor(8,1);
  lcd.print("H: ");
  lcd.setCursor(11,1);
  lcd.print(tempMax,1);
}

void loop() {
  // LEITURA DO CONVERSOR ANALOGICO
  adc0 = ads.readADC_SingleEnded(SENSORAC_REDE);
  adc1 = ads.readADC_SingleEnded(SENSORAC_CARGA);
  adc2 = ads.readADC_SingleEnded(SENSORDC_BATERIA);
  adc3 = ads.readADC_SingleEnded(SENSORDC_PLACASOLAR);

  // CALCULO PARA DADOS DO CONVERSOR AD
  sensorAcRede = (adc0 * 0.1875)/1000;
  sensorAcCarga = (adc1 * 0.1875)/1000;
  sensorDcBateria = (adc2 * 0.1875)/1000;
  sensorDcPlacaSolar = (adc3 * 0.1875)/1000;

  carregamentoBateria();
  temperatura();
  lcdBacklight();

  Serial.print("SENSOR AC REDE: "); 
  Serial.print(adc0);
  Serial.print("\tVoltage: ");
  Serial.println(sensorAcRede, 7);

  Serial.print("SENSOR AC CARGA: ");
  Serial.print(adc1);
  Serial.print("\tVoltage: ");
  Serial.println(sensorAcCarga, 7);

  Serial.print("SENSOR DC BATERIA: ");
  Serial.print(adc2);
  Serial.print("\tVoltage: ");
  Serial.println(sensorDcBateria, 7);

  Serial.print("SENSOR DC PLACA SOLAR: ");
  Serial.print(adc3);
  Serial.print("\tVoltage: ");
  Serial.println(sensorDcPlacaSolar, 7);
  Serial.println();

  carregamentoBateria();

}

// Paginacao de telas
boolean debounce(boolean last, int pin) {
  boolean current = digitalRead(pin);
  if (last != current) {
    delay(5);
    current = digitalRead(pin);
  }
  return current;
}
