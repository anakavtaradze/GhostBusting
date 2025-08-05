  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
/*#include "main.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
*/
/*#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"
#include "stdio.h"

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)

int counter = 1;

int main(void)
{
	//initialise_monitor_handles();*/ // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//	HAL_Init();

	/* Peripheral initializations using BSP functions */
//	BSP_ACCELERO_Init();
//	BSP_LED_Init(LED2);
//	BSP_MAGNETO_Init(); // initializing magnetometer
//	BSP_HSENSOR_Init(); // initializing humidity sensor
//	int tickstart = HAL_GetTick();

//	while (1)
//	{
//		if(counter==2){

//			float accel_data[3];
//			int16_t accel_data_i16[3] = { 0 };
//			BSP_MAGNETO_GetXYZ(accel_data_i16);// array to store the x, y and z readings.
			//BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
			// the function above returns 16 bit integers which are acceleration in mg (9.8/1000 m/s^2).
			// Converting to float to print the actual acceleration.

			//accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
//			accel_data[0] = (float)accel_data_i16[0] * (10000.0f);
//			accel_data[1] = (float)accel_data_i16[1] * (10000.0f);
//			accel_data[2] = (float)accel_data_i16[2] * (10000.0f);

			//printf("Accel X : %f; Accel Y : %f; Accel Z : %f\n", accel_data[0], accel_data[1], accel_data[2]);
//			printf("Magneto X : %f; Magneto Y : %f; Magneto Z : %f\n", accel_data[0], accel_data[1], accel_data[2]);

/*		}

		if (counter==3){

			float temp_data;
					temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor

					printf("Temperature : %f\n", temp_data);
			counter=0;
		}


		float h_data;
		h_data = BSP_HSENSOR_ReadHumidity();
		printf("Humidity : %f\n", h_data);


		if ((HAL_GetTick() - tickstart) >= 500){

			BSP_LED_Toggle(LED2);
			tickstart = HAL_GetTick();
		}

		HAL_Delay(500);	// read once a ~second.
		counter++;


	}


}
*/



/******************************************************************************
* @file           : main.c
* @brief          : Main program body
* (c) EE2028 Teaching Team
******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "wifi.h"
#include <sys/types.h>

#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"

// Defines and constants
#define BUTTON_DEBOUNCE_DELAY 1000
#define UART_BUFFER_SIZE 128
#define ALPHA 0.8f  // Smoothing factor for EMA

#define MAX_LENGTH 400	// adjust it depending on the max size of the packet you expect to send or receive
#define WIFI_READ_TIMEOUT 10000
#define WIFI_WRITE_TIMEOUT 10000

static void UART1_Init(void);
static void MX_GPIO_Init(void);
extern void initialise_monitor_handles(void);
void SystemClock_Config(void);
int time_clicked;
int num_clicked = 1;
int normal_mode = 1;
int configured = 0;

int mode = 1, modeChanged = 1, ghosts_captured = 0;

float mag_data[3];
int16_t mag_data_i16[3] = { 0 };
float accel_data[3];
int16_t accel_data_i16[3] = { 0 };
float gyro_data[3];
int16_t gyro_data_il6[3] = { 0 };
float t_data = 28.0;
float h_data = 50.0;
float p_data = 1000.0;

float magneto_config;
float temp_config;
float h_config;
float accel_config[3];
float p_config;
float gyro_config[3];




int not_clicked = 1;

int wifi = 0;
const char* WiFi_SSID = "Anna";
const char* WiFi_password = "Ana123!!!";
const WIFI_Ecn_t WiFi_security = WIFI_ECN_WPA2_PSK;
const uint16_t SOURCE_PORT = 1234;

uint8_t ipaddr[4] = {192, 168, 137, 1};

WIFI_Status_t WiFi_Stat; // WiFi status. Should remain WIFI_STATUS_OK if everything goes well

#ifdef USING_IOT_SERVER
	const char* SERVER_NAME = "demo.thingsboard.io"; 	// domain name of the IoT server used
	const uint16_t DEST_PORT = 80;			// 'server' port number. Change according to application layer protocol. 80 is the destination port for HTTP protocol.
#else
	const uint16_t DEST_PORT = 2028;		// 'server' port number - this is the port Packet Sender listens to (as you set in Packer Sender)
												// and should be allowed by the OS firewall
#endif
SPI_HandleTypeDef hspi3;

void Get_Sensor_Readings(){
    BSP_MAGNETO_GetXYZ(mag_data_i16);
    BSP_ACCELERO_AccGetXYZ(accel_data_i16);
    BSP_GYRO_GetXYZ(gyro_data_il6);

    mag_data[0] = ALPHA * (float)mag_data_i16[0] + (1 - ALPHA) * mag_data[0];
    mag_data[1] = ALPHA * (float)mag_data_i16[1] + (1 - ALPHA) * mag_data[1];
    mag_data[2] = ALPHA * (float)mag_data_i16[2] + (1 - ALPHA) * mag_data[2];

    t_data = ALPHA * BSP_TSENSOR_ReadTemp() + (1 - ALPHA) * t_data;
    h_data = ALPHA * BSP_HSENSOR_ReadHumidity() + (1 - ALPHA) * h_data;

    accel_data[0] = ALPHA * (float)accel_data_i16[0] * (9.8/1000.0f) + (1 - ALPHA) * accel_data[0];
    accel_data[1] = ALPHA * (float)accel_data_i16[1] * (9.8/1000.0f) + (1 - ALPHA) * accel_data[1];
    accel_data[2] = ALPHA * (float)accel_data_i16[2] * (9.8/1000.0f) + (1 - ALPHA) * accel_data[2];

    gyro_data[0] = ALPHA * (float)gyro_data_il6[0] + (1 - ALPHA) * gyro_data[0];
    gyro_data[1] = ALPHA * (float)gyro_data_il6[1] + (1 - ALPHA) * gyro_data[1];
    gyro_data[2] = ALPHA * (float)gyro_data_il6[2] + (1 - ALPHA) * gyro_data[2];

    p_data = ALPHA * BSP_PSENSOR_ReadPressure() + (1 - ALPHA) * p_data;
}

//reaction to detecting click of a button & double clicking
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == BUTTON_EXTI13_Pin){
		if(!not_clicked)
			num_clicked++;
		if(HAL_GetTick()-time_clicked<=1000 && num_clicked > 1){
			modeChanged = 1;
			if (normal_mode == 0){
				normal_mode = 1;
				configured = 0;
			}
			else
				normal_mode = 0;
			num_clicked = 0;
		}
		time_clicked = HAL_GetTick();
		not_clicked = 0;
	}
	if (GPIO_Pin == GPIO_PIN_1){
	  SPI_WIFI_ISR();
	}
}


float vec_magnitude(float data[3]) {
    return sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
}


UART_HandleTypeDef huart1;

void Sensors_Init(void){
    BSP_TSENSOR_Init();
    BSP_ACCELERO_Init();
    BSP_MAGNETO_Init();
    BSP_GYRO_Init();
    BSP_HSENSOR_Init();
    BSP_PSENSOR_Init();
}

void Sensors_Calibrate(void) {
 // Calibration
	Get_Sensor_Readings();
	temp_config = t_data;
	h_config = h_data;
	p_config = p_data;
	magneto_config = vec_magnitude(mag_data);
	for (int i=0; i<3; i++){
		accel_config[i] = accel_data[i];
		gyro_config[i] = gyro_data[i];
	}
}

int main(void) {
	/* Reset of all peripherals, Initializes Systick etc. */
	HAL_Init();
	uint8_t req[MAX_LENGTH];	// request packet
	uint8_t resp[MAX_LENGTH];	// response packet
	uint16_t Datalen;

	Sensors_Init();
	BSP_LED_Init(LED2);
	initialise_monitor_handles();
	MX_GPIO_Init();
	UART1_Init();

	int tickstart = HAL_GetTick();

	while (1) {
		char message[UART_BUFFER_SIZE];

		if (modeChanged){
			Switch_Mode();
			modeChanged = 0;
		}

		//normal mode
		if (normal_mode){
			//performs action every 1 second
			if ((HAL_GetTick() - tickstart) >= 1000){
				tickstart = HAL_GetTick();
				Get_Sensor_Readings();
				if(!wifi)
						Print_Sensor_Data();
					else{
						float mag_magnitude = vec_magnitude(mag_data);
						float accel_magnitude = vec_magnitude(accel_data);
						float gyro_magnitude = vec_magnitude(gyro_data);

						sprintf((char*)req, "T: %.2f C, P: %.2f hPa, H: %.2f %% RH, A: %.2f m/s^2, G: %.2f mdeg/s, M: %.2f gauss\r\n\n",t_data, p_data, h_data, accel_magnitude, gyro_magnitude, mag_magnitude);
						WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
					}
			}
		}

		//ghost busting mode
		else{
			if(!configured){
				char message_enter_mode[128];
				sprintf(message_enter_mode, "Please configure your PHANTASMA. Place it in a position you want to catch ghosts from. Type in 'Done' once finished.\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_enter_mode, strlen(message_enter_mode),HAL_MAX_DELAY);
			}

			while(!normal_mode && !configured){
				uint8_t buffer[5];
				buffer[4]="\0";
				HAL_UART_Receive(&huart1,buffer,5,HAL_MAX_DELAY);
				char message_config[UART_BUFFER_SIZE];
				if (strncmp((char*)buffer, "Done\r", 5) == 0) {
					sprintf(message_config, "Message Received! Analyzing...\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_config, strlen(message_config),HAL_MAX_DELAY);

					Sensors_Calibrate();

					sprintf(message_config, "Configuration Complete!\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_config, strlen(message_config),HAL_MAX_DELAY);

					configured = 1;
					break;
				}
				if(buffer[4]!=NULL){
					sprintf(message_config, "Incorrect Message. Please Enter 'Done' once you are ready!\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_config, strlen(message_config),HAL_MAX_DELAY);
					buffer[4] = '0';
				}
			}

			//performs action every 1 second
			if ((HAL_GetTick() - tickstart) >= 1000){
				tickstart = HAL_GetTick();

				printf("busting\n");

				float streshold_magneto_mag = 300;
				float streshold_temp = 0.3;
				float streshold_h = 1;
				float streshold_accel[3] = {0.05, 0.05, 0.05};
				float streshold_p = 0.5;
				float streshold_gyro[3] = {8500, 5, 40000};

				char message[UART_BUFFER_SIZE];

				Get_Sensor_Readings();

				if(fabs(vec_magnitude(mag_data)-magneto_config)>=streshold_magneto_mag){
					not_clicked = 1;
					sprintf(message, "Changes in magnetic field detected!\r\nChange = %0.4f\r\n", vec_magnitude(mag_data)-magneto_config);
					HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

					float starting_magnitude = magneto_config;
					char messageed[UART_BUFFER_SIZE];
					int tickstart1 = HAL_GetTick();

					while(not_clicked){
						if((HAL_GetTick() - tickstart1) >= 1000){
							sprintf(messageed, "Ghost detected in! Prepare to bust!\r\n", vec_magnitude(mag_data)-magneto_config);
							HAL_UART_Transmit(&huart1, (uint8_t*)messageed, strlen(messageed),HAL_MAX_DELAY);
							tickstart1 = HAL_GetTick();
						}
						BSP_MAGNETO_GetXYZ(mag_data_i16);
						mag_data[0] = (float)mag_data_i16[0];
						mag_data[1] = (float)mag_data_i16[1];
						mag_data[2] = (float)mag_data_i16[2];
						float magnitude = vec_magnitude(mag_data);
						float blinking_period = 2000-0.5*fabs(magnitude-starting_magnitude);
						printf("%f\r\n", blinking_period);
						if(blinking_period < 75)
							blinking_period = 75;
						if ((HAL_GetTick() - tickstart) >= blinking_period){
							BSP_LED_Toggle(LED2);
							tickstart = HAL_GetTick();
						}
					}
					sprintf(message, "Ghost Captured!\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message),HAL_MAX_DELAY);
					ghosts_captured++;
					not_clicked = 1;
					BSP_LED_Off(LED2);
					while(not_clicked)
						continue;
					BSP_MAGNETO_GetXYZ(mag_data_i16);
					mag_data[0] = (float)mag_data_i16[0];
					mag_data[1] = (float)mag_data_i16[1];
					mag_data[2] = (float)mag_data_i16[2];
					magneto_config = vec_magnitude(mag_data);
				}
				if(fabs(t_data-temp_config)>=streshold_temp){
					sprintf(message, "Change in temperature detected\n Change = %0.4f\r\n", t_data);
					HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message),HAL_MAX_DELAY);
				}
				if(fabs(h_data-h_config)>=streshold_h){
					sprintf(message, "Change in humidity detected\n Change = %0.4f\r\n", h_data-h_config);
					HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message),HAL_MAX_DELAY);
				}
				if(fabs(accel_data[0]-accel_config[0])>=streshold_accel[0] || fabs(accel_data[1]-accel_config[1])>=streshold_accel[1] || fabs(accel_data[2]-accel_config[2])>=streshold_accel[2]){
					sprintf(message, "Device orientation compromised!!\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message),HAL_MAX_DELAY);
					if(fabs(accel_data[0]-accel_config[0])>=streshold_accel[0]){
						sprintf(message, "Accelerometer Change in X = %0.4f\r\n", accel_data[0]-accel_config[0]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message),HAL_MAX_DELAY);
					}
					if(fabs(accel_data[1]-accel_config[1])>=streshold_accel[0]){
						sprintf(message, "Accelerometer Change in Y = %0.4f\r\n", accel_data[1]-accel_config[1]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message),HAL_MAX_DELAY);
					}
					if(fabs(accel_data[2]-accel_config[2])>=streshold_accel[0]){
						sprintf(message, "Accelerometer Change in Z = %0.4f\r\n", accel_data[2]-accel_config[2]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message),HAL_MAX_DELAY);
					}
				}

				if (fabs(gyro_data[0] - gyro_config[0]) >= streshold_gyro[0] || fabs(gyro_data[1] - gyro_config[1]) >= streshold_gyro[1] || fabs(gyro_data[2] - gyro_config[2]) >= streshold_gyro[2]) {
					sprintf(message, "Device orientation compromised!!\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

					if (fabs(gyro_data[0] - gyro_config[0]) >= streshold_gyro[0]) {
						sprintf(message, "Gyroscope Change in X = %0.4f\r\n", gyro_data[0] - gyro_config[0]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
					}
					if (fabs(gyro_data[1] - gyro_config[1]) >= streshold_gyro[1]) {
						sprintf(message, "Gyroscope Change in Y = %0.4f\r\n", gyro_data[1] - gyro_config[1]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
					}
					if (fabs(gyro_data[2] - gyro_config[2]) >= streshold_gyro[2]) {
						sprintf(message, "Gyroscope Change in Z = %0.4f\r\n", gyro_data[2] - gyro_config[2]);
						HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
					}
				}


				if(fabs(p_data-p_config)>=streshold_p){
					sprintf(message, "Change in pressure detected\r\nChange = %0.4f\r\n", p_data-p_config);
					HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message),HAL_MAX_DELAY);
				}
		        temp_config = t_data;
		        h_config = h_data;
		        p_config = p_data;
		        for (int i=0; i<3; i++){
		        	accel_config[i] = accel_data[i];
		        	gyro_config[i] = gyro_data[i];
		        }
		    }
		}
    }
}


void Switch_Mode() {
	char message[UART_BUFFER_SIZE];
	if (normal_mode) {
		snprintf(message, UART_BUFFER_SIZE, "Entering Normal Mode...\nTotal ghosts captured: %d\r\n", ghosts_captured);
		HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
		WiFi_Stat = WIFI_Init();  // if it gets stuck here, you likely did not include EXTI1_IRQHandler() in stm32l4xx_it.c as mentioned above
		WiFi_Stat &= WIFI_Connect(WiFi_SSID, WiFi_password, WiFi_security); // joining a WiFi network takes several seconds. Don't be too quick to judge that your program has 'hung' :)
		if(WiFi_Stat==WIFI_STATUS_OK)wifi = 1;
		WiFi_Stat = WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, DEST_PORT, SOURCE_PORT); // Make a TCP connection.
		if(WiFi_Stat!=WIFI_STATUS_OK)wifi = 0;
	}
	else {
		snprintf(message, UART_BUFFER_SIZE, "Entering Ghost Busting Mode...\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
	}
}

void Print_Sensor_Data() {
	char buffer[UART_BUFFER_SIZE];
	float mag_magnitude = vec_magnitude(mag_data);
	float accel_magnitude = vec_magnitude(accel_data);
	float gyro_magnitude = vec_magnitude(gyro_data);

	snprintf(buffer, UART_BUFFER_SIZE,"T: %.2f C, P: %.2f hPa, H: %.2f %% RH, A: %.2f m/s^2, G: %.2f mdeg/s, M: %.2f gauss\r\n\n",t_data, p_data, h_data, accel_magnitude, gyro_magnitude, mag_magnitude);
	HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

static void UART1_Init(void){
	/* Pin configuration for UART. BSP_COM_Init() can do this automatically */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Configuring UART1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	while (HAL_UART_Init(&huart1) != HAL_OK){
		continue;
	}
}

static void MX_GPIO_Init(void){
	__HAL_RCC_GPIOC_CLK_ENABLE();  // Enable AHB2 Bus for GPIOC

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Enable NVIC EXTI line 13
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void SPI3_IRQHandler(void)
{
	HAL_SPI_IRQHandler(&hspi3);
}



/* (c) EE2028, ECE, NUS */

/*#include "main.h"
#include "stdio.h"
#include "wifi.h"
#include <stdlib.h>	// for rand(). Can be removed if valid sensor data is sent instead

#define MAX_LENGTH 400	// adjust it depending on the max size of the packet you expect to send or receive
#define WIFI_READ_TIMEOUT 10000
#define WIFI_WRITE_TIMEOUT 10000
//#define USING_IOT_SERVER // This line should be commented out if using Packet Sender (not IoT server connection)

const char* WiFi_SSID = "Anna";				// Replacce mySSID with WiFi SSID for your router / Hotspot
const char* WiFi_password = "Ana123!!!";	// Replace myPassword with WiFi password for your router / Hotspot
const WIFI_Ecn_t WiFi_security = WIFI_ECN_WPA2_PSK;	// WiFi security your router / Hotspot. No need to change it unless you use something other than WPA2 PSK
const uint16_t SOURCE_PORT = 1234;	// source port, which can be almost any 16 bit number

uint8_t ipaddr[4] = {192, 168, 137, 1}; // IP address of your laptop wireless lan adapter, which is the one you successfully used to test Packet Sender above.
									// If using IoT platform, this will be overwritten by DNS lookup, so the values of x and y doesn't matter
											//(it should still be filled in with numbers 0-255 to avoid compilation errors)

#ifdef USING_IOT_SERVER
	const char* SERVER_NAME = "demo.thingsboard.io"; 	// domain name of the IoT server used
	const uint16_t DEST_PORT = 80;			// 'server' port number. Change according to application layer protocol. 80 is the destination port for HTTP protocol.
#else
	const uint16_t DEST_PORT = 2028;		// 'server' port number - this is the port Packet Sender listens to (as you set in Packer Sender)
												// and should be allowed by the OS firewall
#endif
SPI_HandleTypeDef hspi3;

int main(void)
{
	printf("Hello");
  HAL_Init();

  uint8_t req[MAX_LENGTH];	// request packet
  uint8_t resp[MAX_LENGTH];	// response packet
  uint16_t Datalen;
  WIFI_Status_t WiFi_Stat; // WiFi status. Should remain WIFI_STATUS_OK if everything goes well

  WiFi_Stat = WIFI_Init();						// if it gets stuck here, you likely did not include EXTI1_IRQHandler() in stm32l4xx_it.c as mentioned above
  WiFi_Stat &= WIFI_Connect(WiFi_SSID, WiFi_password, WiFi_security); // joining a WiFi network takes several seconds. Don't be too quick to judge that your program has 'hung' :)
  if(WiFi_Stat!=WIFI_STATUS_OK) while(1); 					// halt computations if a WiFi connection could not be established.

#ifdef USING_IOT_SERVER
  WiFi_Stat = WIFI_GetHostAddress(SERVER_NAME, ipaddr);	// DNS lookup to find the ip address, if using a connection to an IoT server
#endif
  // WiFi_Stat = WIFI_Ping(ipaddr, 3, 200);					// Optional ping 3 times in 200 ms intervals
  WiFi_Stat = WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, DEST_PORT, SOURCE_PORT); // Make a TCP connection.
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  // "conn" is just a name and serves no functional purpose

  if(WiFi_Stat!=WIFI_STATUS_OK) while(1); 					// halt computations if a connection could not be established with the server

  while (1)
  {
	  int temper = rand()%40; // Just a random value for demo. Use the reading from sensors as appropriate
#ifdef USING_IOT_SERVER
	  char json[100];	// change the size to the max size of the JSON data
	  sprintf((char*)json,"{\"temperature\":%3d}",temper);
	  sprintf((char*)req, "POST /api/v1/myAPIkey/telemetry HTTP/1.1\r\nHost: demo.thingsboard.io\r\nContent-Length: %d\r\n\r\n%s",strlen(json),json);
	  //Important : Replace myAPIkey with your API key.
#else
	  sprintf((char*)req, "temperature : %d\r", temper);
#endif
	  WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);

#ifdef USING_IOT_SERVER
      WiFi_Stat = WIFI_ReceiveData(1, resp, MAX_LENGTH, &Datalen, WIFI_READ_TIMEOUT); // Get response from the server.
      // This can also be used with PacketSender if you wish to receive a response. However, make sure that you send the response fast enough that it won't time out.
      // This function will block until a response is received, or timeout occurs, whichever is earlier.
      // You can print the server response to UART (TeraTerm) if necessary for debugging purposes.
      // Thingsboard will send a response similiar to the one below if the telemetry it received is good.
      /* HTTP/1.1 200
         Content-Length: 0
         Date: Sun, 01 Nov 2020 10:25:18 GMT */
/*      resp[Datalen] = '\0'; // to null terminate the response string
      if( (Datalen == 0) || !strstr((char *)resp, "200") ) while(1);
      // halt computations if the server does not respond (see below) or if the server sends a bad response (likely a wrong API key)
      // Thingsboard server closes connection a few seconds after a previous telemetery. Keep this in mind if you pause within this main while() loop.
#endif
	  HAL_Delay(1000);

  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin){
  case GPIO_PIN_1:
	  SPI_WIFI_ISR();
	  break;
  } // use more cases in the case statement for other EXTI interrupts
}

void SPI3_IRQHandler(void)
{
	HAL_SPI_IRQHandler(&hspi3);
}
*/
