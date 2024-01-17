/*
 Nome ALUNO A - José Catarino
 Nome ALUNO B - Filipe Letras
 IPLEIRIA - Instituto Politécnico de Leiria
 ESTG - Escola Superior de Tecnologia e Gestão
 LEAU- Licenciatura em Engenharia Automóvel
 SEEV - Sistemas Elétricos e Eletrónicos de Veículos

 TP2: Pretende-se neste trabalho prático a implementação de um algoritmo que obtenha os valores de um
 potenciómetro e coordenadas GPS. Estes dados serão adquiridos e processados por um Microcontrolador do
 tipo ESP32, e mostrados num ecrã tipo TFT, de modo a caracterizar o pavimento com diferentes
 intensidades e cores de LED.

 LINK YOUTUBE: https://www.youtube.com/watch?v=f_vLrGs9cXw

*/

//Inclusão das bibiliotecas
#include "Arduino.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include "Adafruit_ST7735.h"
#include <TinyGPS++.h>

//Debug do Core 1
//#define DEBUG_PRINT1

//Debug do Core 0
//#define DEBUG_PRINT0

//Configuração do ADC
#define ADC_1_0 36
#define ADC_RESOLUTION 12
#define VREF_PLUS 3.3
#define VREF_MINUS 0.0

//Pinos do LED RGB
#define RED_PIN 12
#define GREEN_PIN 14
#define BLUE_PIN 13

//Configuração do GPS
#define GPS_BAUDRATE 9600
TinyGPSPlus gps;

//Criação da estrutura para os dados enviados pelo GPS
typedef struct {
	int erro;
	float latitude;
	float longitude;
	float altitude;
	float velocidade;
	/*int year;
	 int month, day, hour, minute, second; //Acabou por não ser usado*/
} GPSData;

//Configuração do Ecrã TFT
#define TFT_CS 5
#define TFT_RST 22
#define TFT_DC 4
#define TFT_MOSI 23
#define TFT_CLK 18

//Funções das tarefas
void vTaskFunctionADC(void *pvParameters);
void vTaskFunctionGPS(void *pvParameters);
void vTaskFunctionApresentacao(void *pvParameters);
void vTaskFunctionLCD(void *pvParameters);
void vTaskFunctionBrain(void *pvParameters);

//Criação do objecto para o TFT
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK,
TFT_RST);

//Criação das Queues e Semáforos
QueueHandle_t xQueue_GPS, xQueue_ADC, xQueue_BRAIN;
SemaphoreHandle_t xSemaphore; // Permite o acesso ao estado de anomalia

//Definição do valor mínimo que causa uma anomalia
int diff_anomalia = 1000;

//Definição dos valores para a intensidade
int int_baixa = 1500, int_media = 2000, int_alta = 2500;

/*-------------------------------------CONFIGURAÇÃO-------------------------------------*/

void setup() {

	//Inicialização das portas série
	Serial.begin(115200);
	Serial2.begin(GPS_BAUDRATE);

#ifdef DEBUG_PRINT1
	Serial.println("--- Inicio Setup ---");
#endif

	//Definição da tarefa setup para a prioridade máxima disponível
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

	//Configuração do ADC
	analogReadResolution(ADC_RESOLUTION);

	//Inicialização do ecrã
	tft.initR(INITR_BLACKTAB);
	//Rotação do ecrã igual a 0 graus
	tft.setRotation(1);

	//Definição dos pinos do LED como output (saída)
	pinMode(RED_PIN, OUTPUT);
	pinMode(GREEN_PIN, OUTPUT);
	pinMode(BLUE_PIN, OUTPUT);
	//Estado inicial do LED é desligado
	digitalWrite(RED_PIN, LOW);
	digitalWrite(GREEN_PIN, LOW);
	digitalWrite(BLUE_PIN, LOW);

	//Criação das Queues
	xQueue_GPS = xQueueCreate(1, sizeof(GPSData));
	xQueue_ADC = xQueueCreate(1, sizeof(int));
	xQueue_BRAIN = xQueueCreate(1, sizeof(int));

	//Criação do Semáforo
	xSemaphore = xSemaphoreCreateBinary();

	//Criação das Tarefas
	xTaskCreatePinnedToCore(vTaskFunctionApresentacao, "Task Apresentacao",
			4096,
			NULL, 5, NULL, 1);

	xTaskCreatePinnedToCore(vTaskFunctionBrain, "Task Brain", 1024, NULL, 4,
	NULL, 1);

	xTaskCreatePinnedToCore(vTaskFunctionADC, "Task ADC", 1024, NULL, 1, NULL,
			0);
	xTaskCreatePinnedToCore(vTaskFunctionGPS, "Task GPS", 2048, NULL, 3, NULL,
			1);

	xTaskCreatePinnedToCore(vTaskFunctionLCD, "Task LCD", 4096,
	NULL, 2, NULL, 1);

#ifdef DEBUG_PRINT1
	Serial.println("--- Fim Setup ---");
#endif
}

/*-----------------------------------------------------------------------------------*/

/*-------------Tarefa responsável pela apresentação do trabalho e alunos-------------*/

void vTaskFunctionApresentacao(void *pvParameters) {
#ifdef DEBUG_PRINT1
	Serial.println("--- Inicio Apresentacao ---");
#endif

	TickType_t xLastWakeTime;

	//Define o fundo do ecrã como preto, a cor da letra como verde e o tamanho como 1
	tft.fillScreen(ST7735_BLACK);
	tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
	tft.setTextSize(1);

	//Mostra a informação
	tft.setCursor(5, 5);
	tft.print("Instituto Politecnico");
	tft.setCursor(5, 20);
	tft.print("de Leiria");
	tft.setCursor(5, 35);
	tft.print("Engenharia Automovel");
	tft.setCursor(5, 50);
	tft.print("Sistemas Eletricos e");
	tft.setCursor(5, 65);
	tft.print("Eletronicos Veiculos ");
	tft.setCursor(5, 80);
	tft.print("Trabalho realizado por:");
	tft.setCursor(5, 95);
	tft.print("Jose Catarino - 2191570");
	tft.setCursor(5, 110);
	tft.print("Filipe Letras - 2192504");

	//Ciclo para aumentar o tempo da apresentação
	for (int index_i = 0; index_i < 5000; index_i++) {
		tft.setCursor(0, 0);
		tft.setTextColor(ST7735_BLACK, ST7735_BLACK);
		tft.print(".");
	}

	//Aguarda 1s
	vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_PERIOD_MS));

	//Limpa o ecrã
	tft.fillScreen(ST7735_BLACK);

#ifdef DEBUG_PRINT1
	Serial.println("--- Fim Apresentacao ---");
#endif

	//Mata a tarefa
	vTaskDelete(NULL);

}

/*-------------Tarefa responsável por lidar com a deteção de anomalias-------------*/

void vTaskFunctionBrain(void *pvParameters) {
#ifdef DEBUG_PRINT1
	Serial.println("--- Inicio Brain ---");
#endif

	TickType_t xLastWakeTime;
	int diff, val_anomalia;

	for (;;) {

		//Verifica se existe um valor na queue do ADC
		if (xQueueReceive(xQueue_ADC, &diff, 0) == pdTRUE) {

			//Envia o valor da diferença para a queue do Brain
			xQueueSendToBack(xQueue_BRAIN, &val_anomalia, 0);

#ifdef DEBUG_PRINT1
		Serial.print("Diferenca: ");
		Serial.println(diff);
#endif

			//Dá o semáforo indicando que houve uma anomalia
			xSemaphoreGive(xSemaphore);
		}

		//Repete a tarefa com uma periocidade de 200ms
		vTaskDelayUntil(&xLastWakeTime, (200 / portTICK_PERIOD_MS));
	}

#ifdef DEBUG_PRINT1
	Serial.println("--- Fim Brain ---");
#endif
}

/*-------------Tarefa responsável por ler os valores do ADC-------------*/

void vTaskFunctionADC(void *pvParameters) {
#ifdef DEBUG_PRINT0
	Serial.println("--- Inicio ADC ---");
#endif

	TickType_t xLastWakeTime;
	int ADC_Valor_1 = 0;
	int ADC_Valor_2 = 0;
	int diff = 0;

	//Valor inicial do ADC
	ADC_Valor_2 = analogRead(ADC_1_0);

	for (;;) {

		//Lê o primeiro valor do ADC
		ADC_Valor_1 = ADC_Valor_2;

		//Repete a tarefa com uma periocidade de 50ms
		vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_PERIOD_MS));

		//Lê o segundo valor do ADC
		ADC_Valor_2 = analogRead(ADC_1_0);

#ifdef DEBUG_PRINT0
		Serial.println(ADC_Valor_1);
#endif

#ifdef DEBUG_PRINT0
		Serial.println(ADC_Valor_2);
#endif

		//Cálculo da diferença absoluta
		diff = abs(ADC_Valor_1 - ADC_Valor_2);

		//Verifica se esta diferença é superior ao valor mínimo definido
		if (diff > diff_anomalia) {

			//Se for superior envia o valor da diferença para a queue
			xQueueSendToBack(xQueue_ADC, &diff, 0);
		}

	}

#ifdef DEBUG_PRINT0
	Serial.println("--- Fim ADC ---");
#endif
}

/*-------------Tarefa responsável por processar os dados fornecidos pelo GPS-------------*/

void vTaskFunctionGPS(void *pvParameters) {
#ifdef DEBUG_PRINT1
	Serial.println("--- Inicio GPS ---");
#endif

	TickType_t xLastWakeTime;
	GPSData xSendStructure;
	int erro = 0;
	float latitude = 0.0, longitude = 0.0, altitude = 0.0, velocidade = 0.0;

	for (;;) {

		//Se o módulo GPS estiver a funcionar, lê e regista os valores
		if (Serial2.available() > 0) {
			if (gps.encode(Serial2.read())) {
				if (gps.location.isValid()) {
					latitude = gps.location.lat();
					longitude = gps.location.lng();
					altitude = gps.altitude.meters();

					if (gps.speed.isValid()) {
						velocidade = gps.speed.kmph();
					}
				}
			}
		}
		//Se o módulo de GPS não obtiver resposta cria um erro
		if (millis() > 5000 && gps.charsProcessed() < 10) {
			erro = 1;
		}

		//Preenche a estrutura com a informação do GPS
		xSendStructure.erro = erro;
		xSendStructure.latitude = latitude;
		xSendStructure.longitude = longitude;
		xSendStructure.altitude = altitude;
		xSendStructure.velocidade = velocidade;

		//Envia a estrutura para a queue
		xQueueSendToBack(xQueue_GPS, &xSendStructure, 0);

#ifdef DEBUG_PRINT1
		Serial.println(xSendStructure.erro);
		Serial.println(xSendStructure.latitude);
		Serial.println(xSendStructure.longitude);
		Serial.println(xSendStructure.altitude);
		Serial.println(xSendStructure.velocidade);
#endif

		//Repete a tarefa com uma periocidade de 100ms
		vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_PERIOD_MS));

	}
#ifdef DEBUG_PRINT1
	Serial.println("--- Fim GPS ---");
#endif
}

/*-------------Tarefa responsável por mostrar toda a informação no ecrã-------------*/

void vTaskFunctionLCD(void *pvParameters) {
#ifdef DEBUG_PRINT1
	Serial.println("--- Inicio LCD ---");
#endif

	TickType_t xLastWakeTime;
	GPSData xReceivedStructure;
	int val_anomalia;

	for (;;) {

		//Define a rotação do ecrã para 90 graus
		tft.setRotation(2);

		//Verifica se existe um semáforo

		if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(0)) == pdTRUE) {

			//Mostra que existe uma anomalia
			tft.fillRect(0, 90, 160, 80, ST7735_YELLOW);
			tft.setTextColor(ST7735_BLACK, ST7735_YELLOW);

			//Tamanho do texto definido para 2 para uma melhor leitura
			tft.setTextSize(2);
			tft.setCursor(10, 100);
			tft.print("Anomalia");
			tft.setCursor(10, 120);
			tft.print("Detetada");
			tft.setTextSize(1);
			tft.setCursor(10, 140);
			tft.print("Intensidade: ");

			//Recebe o valor da diferença da queue
			xQueueReceive(xQueue_BRAIN, &val_anomalia, 0);

			//Muda a cor do LED e escreve a intensidade
			if (val_anomalia <= int_baixa) {
				analogWrite(RED_PIN, 0);
				analogWrite(GREEN_PIN, 0);	// Azul
				analogWrite(BLUE_PIN, 100);
				tft.setCursor(82, 140);
				tft.print("Baixa");
			} else if (val_anomalia <= int_media) {
				analogWrite(RED_PIN, 255);
				analogWrite(GREEN_PIN, 200); // Amarelo
				analogWrite(BLUE_PIN, 0);
				tft.setCursor(82, 140);
				tft.print("Media");
			} else if (val_anomalia <= int_alta) {
				analogWrite(RED_PIN, 255);
				analogWrite(GREEN_PIN, 80); // Laranja
				analogWrite(BLUE_PIN, 0);
				tft.setCursor(82, 140);
				tft.print("Alta");
			} else {
				analogWrite(RED_PIN, 255);
				analogWrite(GREEN_PIN, 0); // Vermelho
				analogWrite(BLUE_PIN, 0);
				tft.setCursor(82, 140);
				tft.print("Critica");
			}

		}

		//Se não houver anomalia o LED acende a verde
		else {
			analogWrite(RED_PIN, 0);
			analogWrite(GREEN_PIN, 100); // Verde
			analogWrite(BLUE_PIN, 0);
		}

		//Recebe a estrutura da queue
		xQueueReceive(xQueue_GPS, &xReceivedStructure, 0);

#ifdef DEBUG_PRINT1
		Serial.println(xReceivedStructure.erro);
		Serial.println(xReceivedStructure.latitude);
		Serial.println(xReceivedStructure.longitude);
		Serial.println(xReceivedStructure.altitude);
		Serial.println(xReceivedStructure.velocidade);
#endif

		//Se houver uma má ligação do GPS entra neste if
		if (xReceivedStructure.erro == 1) {
			//Mostra uma mensagem de erro no ecrã
			  tft.fillScreen(ST7735_BLACK);
			  tft.setTextColor(ST7735_RED);
			  tft.setTextSize(2);
			  tft.setCursor(10, 40);
			  tft.print("VERIFICAR LIGACAO");
			  tft.setCursor(10, 90);
			  tft.print("REINICIAR");

			//Cria um loop infinito com o LED vermelho a piscar
			while (1) {
				analogWrite(RED_PIN, 255);
				analogWrite(GREEN_PIN, 0);
				analogWrite(BLUE_PIN, 0);
				delay(500);  //Fica acesso 500ms

				analogWrite(RED_PIN, 0);
				analogWrite(GREEN_PIN, 0);
				analogWrite(BLUE_PIN, 0);
				delay(500);  //Fica apagado 500ms
			}
		}

		else {

			//Mostra a informação do GPS no ecrã
			tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
			tft.setTextSize(1);

			tft.setCursor(2, 5);
			tft.print("Latitude: ");
			tft.print(xReceivedStructure.latitude);

			tft.setCursor(2, 20);
			tft.print("Longitude: ");
			tft.print(xReceivedStructure.longitude);

			tft.setCursor(2, 35);
			tft.print("Altitude: ");
			tft.print(xReceivedStructure.altitude);
			tft.print(" m");

			tft.setCursor(2, 50);
			tft.print("Velocidade: ");
			tft.print(xReceivedStructure.velocidade);
			tft.print(" km/h");
		}

		for (int index_i = 0; index_i < 3000; index_i++) {
			tft.setCursor(128, 0);
			tft.setTextColor(ST7735_BLACK, ST7735_BLACK);
			tft.print(".");
		}

		//Limpa o ecrã
		tft.fillRect(0, 0, 128, 70, ST7735_BLACK);
		tft.fillRect(0, 90, 160, 80, ST7735_BLACK);

		//Repete a tarefa com uma periocidade de 200ms
		vTaskDelayUntil(&xLastWakeTime, (200 / portTICK_PERIOD_MS));
	}
#ifdef DEBUG_PRINT1
	Serial.println("--- Fim LCD ---");
#endif
}

/*---------------------------------------------------------------------*/

void loop() {
	vTaskDelete(NULL);
}
