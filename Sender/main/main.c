#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include "string.h"

#include "esp_log.h"
#include "lora.h"
#include "esp_sleep.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "sht30.h"
#include "sds011.h"

#define MQ7          				ADC1_CHANNEL_3   //gpio num 39 
#define MQ4          				ADC1_CHANNEL_6   //gpio num 34
#define PM25          				ADC1_CHANNEL_7   //gpio num 35
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF

/** Sensor UART configuration. Adapt to your setup. */
#define SDS011_UART_PORT 			UART_NUM_2
#define SDS011_RX_GPIO 				26
#define SDS011_TX_GPIO 				27

/** Time in seconds to let the SDS011 run before taking the measurment. */
#define SDS011_ON_DURATION 			15

/** Interval to read and print the sensor values in seconds. Must be bigger than
 * SDS011_ON_DURATION. */
#define PRINT_INTERVAL 				105
static sht3x_t dev;
static esp_adc_cal_characteristics_t adc1_chars;

bool enable_sleep = false;
bool enable_lora_send = false;
struct Data 
{
	float TP;
	float HM;
	float GAS;
	float D;
	float D10;
	float CO;
};

struct Data data;

////////////////////////////SDS011///////////////////////////////////////
static const struct sds011_tx_packet sds011_tx_sleep_packet = {
    .head = SDS011_PACKET_HEAD,
    .command = SDS011_CMD_TX,
    .sub_command = SDS011_TX_CMD_SLEEP_MODE,
    .payload_sleep_mode = {.method = SDS011_METHOD_SET,
                           .mode = SDS011_SLEEP_MODE_ENABLED},
    .device_id = SDS011_DEVICE_ID_ALL,
    .tail = SDS011_PACKET_TAIL};

static const struct sds011_tx_packet sds011_tx_wakeup_packet = {
    .head = SDS011_PACKET_HEAD,
    .command = SDS011_CMD_TX,
    .sub_command = SDS011_TX_CMD_SLEEP_MODE,
    .payload_sleep_mode = {.method = SDS011_METHOD_SET,
                           .mode = SDS011_SLEEP_MODE_DISABLED},
    .device_id = SDS011_DEVICE_ID_ALL,
    .tail = SDS011_PACKET_TAIL};

void data_task(void* pvParameters) {
	struct sds011_rx_packet rx_packet;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		/** Wake the sensor up. */
		sds011_send_cmd_to_queue(&sds011_tx_wakeup_packet, 0);
		/** Give it a few seconds to create some airflow. */
		vTaskDelay(pdMS_TO_TICKS(SDS011_ON_DURATION * 1000));

		/** Read the data (which is the latest when data queue size is 1). */
		if (sds011_recv_data_from_queue(&rx_packet, 0) == SDS011_OK) {

			float pm2_5;
			float pm10;

			pm2_5 = ((rx_packet.payload_query_data.pm2_5_high << 8) |
			rx_packet.payload_query_data.pm2_5_low) /10.0;
			pm10 = ((rx_packet.payload_query_data.pm10_high << 8) |
			rx_packet.payload_query_data.pm10_low) /10.0;

			ESP_LOGI("task_sds011", "PM2.5: %.2f PM10: %.2f\n",pm2_5, pm10);
			data.D=pm2_5;
			data.D10=pm10;
			enable_lora_send = true;
			/** Set the sensor to sleep. */
			sds011_send_cmd_to_queue(&sds011_tx_sleep_packet, 0);
			ESP_LOGI("Sleep", "Send cmd to sds011 sleep\n");
			vTaskDelay(1000/ portTICK_PERIOD_MS);
			enable_sleep = true;
			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PRINT_INTERVAL * 1000));
		}
	}
}
////////////////////////////////////////////////////////////////////////////

static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW("ADC", "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW("ADC", "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    } else {
        ESP_LOGE("ADC", "Invalid arg");
    }

    return cali_enable;
}
void task_adc(void *pvParameters){
    bool cali_enable = adc_calibration_init();

    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(MQ7, ADC_EXAMPLE_ATTEN));

	gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_NUM_32, 1);
    while (1) {
		gpio_set_level(GPIO_NUM_32, 0);
        
		float a0 = adc1_get_raw(MQ4); // get raw reading from sensor
		float v_o = a0 /( 5 * 1024); // convert reading to volts
		float R_S = (5-v_o) * 1000 / v_o; // apply formula for getting RS
		float GAS = pow(R_S/945,-2.95) * 1000*1000;
		data.GAS=GAS;
		ESP_LOGI("task_adc", "======GAS======");
        ESP_LOGI("task_adc", "GAS: %f", GAS);

		float sensorValue = adc1_get_raw(MQ7);
		float sensor_volt = sensorValue/(1024*3.3);
		float RS_gas = (3.3-sensor_volt)/sensor_volt;
		float ratio = RS_gas/7200; //Replace R0 with the value found using the sketch above
		float x = 1538.46 * ratio;
		float CO = pow(x,-1.709)*1000;
		data.CO=CO;

		ESP_LOGI("task_adc", "======CO======");
		ESP_LOGI("task_adc", "CO %f ", CO);
	
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }
}


void task_sht30(void *arg){
    esp_err_t res;

    // Start periodic measurements with 1 measurement per second.
    ESP_ERROR_CHECK(sht3x_start_measurement(&dev, SHT3X_PERIODIC_1MPS, SHT3X_HIGH));

    // Wait until first measurement is ready (constant time of at least 30 ms
    // or the duration returned from *sht3x_get_measurement_duration*).
    vTaskDelay(sht3x_get_measurement_duration(SHT3X_HIGH));

    TickType_t last_wakeup = xTaskGetTickCount();

    while (1)
    {
        // Get the values and do something with them.
        if ((res = sht3x_get_results(&dev, &data.TP, &data.HM)) == ESP_OK)
			ESP_LOGI("task_sht30","SHT3x Sensor: %.2f °C, %.2f %%\n", data.TP, data.HM);
        else
			ESP_LOGI("task_sht30","Could not get results: %d (%s)", res, esp_err_to_name(res));

        // Wait until 2 seconds (cycle time) are over.
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(10000));
    }
}

void task_tx(void *data)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start");
	uint8_t buf[255]; // Maximum Payload size of SX1276/77/78/79 is 255
    struct Data *data_parsed;
	data_parsed = data;
	int ID=1;
	while(1) {
		if (enable_lora_send) {
			int send_len = sprintf((char *)buf,"{\"ID\":\"%d\",\"CO\":\"%.2f\",\"GAS\":\"%.2f\",\"H\":\"%.2f\",\"T\":\"%.2f\",\"D\":\"%.2f\",\"D10\":\"%.2f\"}",
											ID,data_parsed->CO,data_parsed->GAS,data_parsed->HM,data_parsed->TP,data_parsed->D,data_parsed->D10);
			ESP_LOGI(pcTaskGetName(NULL), "Sending");
			lora_send_packet(buf, send_len);
			vTaskDelay(500/ portTICK_PERIOD_MS);
			lora_send_packet(buf, send_len);
			vTaskDelay(1000/ portTICK_PERIOD_MS);
		}
		
		/** Wait for next interval time. */
		// if (enable_sleep) {
		// 	const int wakeup_time_sec = 120; 
		// 	enable_sleep = false;
		// 	esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);
		// 	ESP_LOGI("Sleep", "Entering lora deep sleep\n");
		// 	lora_sleep();
		// 	esp_deep_sleep_start();
		// }
			
	}
}
void app_main()
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

	/** Initialize the SDS011. */
	sds011_begin(SDS011_UART_PORT, SDS011_TX_GPIO, SDS011_RX_GPIO);

	ESP_ERROR_CHECK(i2cdev_init());
	memset(&dev, 0, sizeof(sht3x_t));

	ESP_ERROR_CHECK(sht3x_init_desc(&dev, 0x44, 0, 4, 5));
	ESP_ERROR_CHECK(sht3x_init(&dev));
	
	assert(xTaskCreatePinnedToCore(data_task, "sds011", 2048, NULL, 2, NULL, 1) == pdTRUE);
	xTaskCreate(&task_sht30, "task_sht30" ,4096, NULL, 6, NULL);
	xTaskCreate(&task_adc, "task_adc", 4096, NULL, 4, NULL);
	xTaskCreate(&task_tx, "task_tx", 4096, &data, 0, NULL);
}