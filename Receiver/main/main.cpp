
#include <iostream>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "cJSON.h"

#include "jsoncpp/value.h"
#include "jsoncpp/json.h"

#include "esp_firebase/app.h"
#include "esp_firebase/rtdb.h"

#include "wifi_utils.h"

#include "firebase_config.h"

#include "lora.h"
#include "lcd.h"

#define I2C_MASTER_SCL_IO 5 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 4 /*!< gpio number for I2C master data  */
#define ESP_SLAVE_ADDR 0x27  /*!< ESP32 slave address, you can set any 7bit value */

using namespace ESPFirebase;

extern fb_data_t data_get;
extern fb_get_data data_node1;
extern fb_get_data data_node2;

void JSON_Analyze(const cJSON *const root)
{
	cJSON *current_element = NULL;
	cJSON_ArrayForEach(current_element, root)
	{
		if (current_element->string)
		{
			const char *string = current_element->string;
			ESP_LOGI("JSON", "[%s]", string);
			if(strcmp(string,"ID")==0){
				if (cJSON_IsString(current_element))
				{
					char *valuestring = current_element->valuestring;
					data_get.ID=atoi(valuestring);
					ESP_LOGI("JSON", "%d", data_get.ID);
				}
			}
			if(strcmp(string,"CO")==0){
				if (cJSON_IsString(current_element))
				{
					char *valuestring = current_element->valuestring;
					data_get.CO=atof(valuestring);
					ESP_LOGI("JSON", "%f", data_get.CO);
				}
			}
			if(strcmp(string,"GAS")==0){
				if (cJSON_IsString(current_element))
				{
					char *valuestring = current_element->valuestring;
					data_get.GAS=atof(valuestring);
					ESP_LOGI("JSON", "%f", data_get.GAS);
				}
			}
			if(strcmp(string,"H")==0){
				if (cJSON_IsString(current_element))
				{
					char *valuestring = current_element->valuestring;
					data_get.H=atof(valuestring);
					ESP_LOGI("JSON", "%f", data_get.H);
				}
			}
			if(strcmp(string,"T")==0){
				if (cJSON_IsString(current_element))
				{
					char *valuestring = current_element->valuestring;
					data_get.T=atof(valuestring);
					ESP_LOGI("JSON", "%f", data_get.T);
				}
			}
			if(strcmp(string,"D")==0){
				if (cJSON_IsString(current_element))
				{
					char *valuestring = current_element->valuestring;
					data_get.D=atof(valuestring);
					ESP_LOGI("JSON", "%f", data_get.D);
				}
			}
			if(strcmp(string,"D10")==0){
				if (cJSON_IsString(current_element))
				{
					char *valuestring = current_element->valuestring;
					data_get.D10=atof(valuestring);
					ESP_LOGI("JSON", "%f", data_get.D10);
				}
			}
		}
	}
}

void task_rx(void *param)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start");
	uint8_t buf[256];
	while (1)
	{
		lora_receive();
		if (lora_received())
		{
			int receive_len = lora_receive_packet(buf, sizeof(buf));
			ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s]", receive_len, receive_len, buf);
			ESP_LOGI("RX", "Deserialize.....");
			cJSON *root2 = cJSON_Parse((char *)buf);
			JSON_Analyze(root2);
			cJSON_Delete(root2);
		}
		vTaskDelay(1);
	}
}
void task_lcd(void * param){
	lcd_init(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, ESP_SLAVE_ADDR, 16, 2, LCD_5x8DOTS);
	lcd_begin();	
	char screen[16];
	char screen1[16];
	while (1)
	{	
		lcd_clear();
		sprintf(screen,"KHANH NGHIEM");
		lcd_setCursor(0, 0);
		lcd_print(screen);
		vTaskDelay(1000 / portTICK_PERIOD_MS);

		//NODE1
		lcd_clear();
		sprintf(screen,"NODE 1");
		lcd_setCursor(0, 0);
		lcd_print(screen);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		lcd_clear();
		sprintf(screen,"T: %.1f  H: %.1f",data_get.T,data_get.H);
		lcd_setCursor(0, 0);
		lcd_print(screen);
		sprintf(screen1,"CO: %.1f",data_get.CO);
		lcd_setCursor(0, 1);
		lcd_print(screen1);
		vTaskDelay(2000 / portTICK_PERIOD_MS);
		
		lcd_clear();
		sprintf(screen,"PM2.5 : %.1f",data_get.D);
		lcd_setCursor(0, 0);
		lcd_print(screen);

		sprintf(screen1,"PM10  : %.1f",data_get.D10);
		lcd_setCursor(0, 1);
		lcd_print(screen1);
		vTaskDelay(2000 / portTICK_PERIOD_MS);	
	}
}
extern "C" void app_main(void)
{
	if (lora_init() == 0) {
		ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
		while(1) {
			vTaskDelay(1);
		}
	}

	ESP_LOGI(pcTaskGetName(NULL), "Frequency is 433MHz");
	lora_set_frequency(433e6); // 433MHz
	lora_enable_crc();

	int cr = 1;
	int bw = 7;
	int sf = 7;

	lora_set_coding_rate(cr);
	//lora_set_coding_rate(CONFIG_CODING_RATE);
	//cr = lora_get_coding_rate();
	ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

	lora_set_bandwidth(bw);
	//lora_set_bandwidth(CONFIG_BANDWIDTH);
	//int bw = lora_get_bandwidth();
	ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

	lora_set_spreading_factor(sf);
	//lora_set_spreading_factor(CONFIG_SF_RATE);
	//int sf = lora_get_spreading_factor();
	ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);
	
	xTaskCreate(&task_rx, "task_rx", 4*1024, NULL, 6, NULL);
    xTaskCreate(&task_lcd, "task_lcd", 4*1024, NULL, 4, NULL);

    wifiInit(SSID, PASSWORD);

    user_account_t account = {USER_EMAIL, USER_PASSWORD};

    FirebaseApp app = FirebaseApp(API_KEY);

    app.loginUserAccount(account);

    RTDB db = RTDB(&app, DATABASE_URL);

    while (1)
    {
        vTaskDelay(5000/portTICK_PERIOD_MS);
        Json::Value data;
		data["T"] = data_get.T;
		data["H"] = data_get.H;
		data["CO"] = data_get.CO;
        data["GAS"] = data_get.GAS;
        data["PM25"] = data_get.D;
        data["PM10"] = data_get.D10;
        db.putData("/node1", data);  
    }
}

