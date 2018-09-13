// Programa: TCC No-Break
// Descricao do projeto


// Portas:
// LCD: Analógica 4 e 5
// Cooler: Digital 7 (representado por LED Vermelho)
// Sensor Temperatura: Digital 3
// Botao Backlight: 2
// AD0: Sensor DC Placa Solar
// AD1: Sensor AC Rede
// Rele CARGA: Digital 6
// Rele Carregamento: Digital 4 IN1 e Digital 5 IN2


// BIBLIOTECAS UTILIZADAS
// Conversor AD
#include <Adafruit_ADS1015.h>

// 
#include <Wire.h>

// LCD I2C
#include <LiquidCrystal_I2C.h>

// Sensor de Temperatura
#include <OneWire.h>
#include <DallasTemperature.h>

// DEFINIÇÕES
// Porta do pino de sinal do DS18B20
#define ONE_WIRE_BUS 3
 
// Define uma instancia do oneWire para comunicacao com o sensor
OneWire oneWire(ONE_WIRE_BUS);
 
// Armazena temperaturas minima e maxima
float tempMin = 999;
float tempMax = 0;

// Define valor do botao
int buttonState = 0;
 
DallasTemperature sensors(&oneWire);
DeviceAddress sensor1;

// Inicializa o display no endereco 0x27
LiquidCrystal_I2C lcd(0x3f,2,1,0,4,5,6,7,3, POSITIVE);

// Endereco Conversor AD
Adafruit_ADS1115 ads(0x48);
float ad0, ad1, ad2, ad3, anag = 0.0;
int16_t adc0, adc1, adc2, adc3, analogica;
 
void setup(void) {
  Serial.begin(9600);

  // Sensor de temperatura
  sensors.begin();

  // Conversor AD (+4 portas)
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

  // inicializa o pino 2 do botao como entrada:
  pinMode(2, INPUT);

  // Rele de carga
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  
  // inicializa pino 4 do COOLER (representado com LED) como saída
  pinMode(7, OUTPUT);

  analogica = analogRead(0);
}
 
void mostra_endereco_sensor(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    // Adiciona zeros se necessário
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void carga() {
  // Leitura tensao da placa DC e compara com valor fixo (18V)
  if (ad0 >= 2.5 || analogica < 0.8) {
    digitalWrite(6, HIGH);
  } else {
    digitalWrite(6, LOW);
  }
}

void carregamentoBateria() {
  // Carregamento da bateria Se tensao placa menor que 12,5 carrega pela rede
  if (ad0 >= 2.5) {
    digitalWrite(4, LOW); // (IN1)
    digitalWrite(5, HIGH);// (IN2)
  } else {
    digitalWrite(5, LOW);
    digitalWrite(4, HIGH);
  }
}

void lcdBacklight() {
  // Acende e apaga backlight do LCD
  // Acao do botao para Backlight
  lcd.setBacklight(LOW);
  // faz a leitura do valor do botao
  buttonState = digitalRead(2);
  // verifica se o botao esta pressionado.
  if (buttonState == HIGH) {
    lcd.setBacklight(HIGH);
  }
  else {
    lcd.setBacklight(LOW);
  }
}

void temperatura() {
  // Le a informacao do sensor
  sensors.requestTemperatures();
  float tempC = sensors.getTempC(sensor1);
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
  // we read from the ADC, we have a sixteen bit integer as a result
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);
  
  ad0 = (adc0 * 0.1875)/1000;
  ad1 = (adc1 * 0.1875)/1000;
  ad2 = (adc2 * 0.1875)/1000;
  ad3 = (adc3 * 0.1875)/1000;
  anag = 
  
  carga();
  carregamentoBateria();
  carga();
  temperatura();
  lcdBacklight();
  carga();

  Serial.print("AIN0: "); 
  Serial.print(adc0);
  Serial.print("\tVoltage: ");
  Serial.println(ad0, 7);

  Serial.print("AIN1: ");
  Serial.print(adc1);
  Serial.print("\tVoltage: ");
  Serial.println(ad1, 7);

  Serial.print("AIN2: ");
  Serial.print(adc2);
  Serial.print("\tVoltage: ");
  Serial.println(ad2, 7);

  Serial.print("AIN3: ");
  Serial.print(adc3);
  Serial.print("\tVoltage: ");
  Serial.println(ad3, 7);
  Serial.println();

  Serial.print("ANAG: ");
  Serial.print(analogica);
  Serial.print("\tVoltage: ");
  Serial.println(adc3, 7);
  Serial.println();
  
  carga();

  carregamentoBateria();

  carga();

  carga();
}
