/*
 * AWS IoT EduKit - Core2 for AWS IoT EduKit
 * Cloud Connected Blinky v1.3.0
 * main.c
 * 
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Additions Copyright 2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
/**
 * @file main.c
 * @brief simple MQTT publish and subscribe for use with AWS IoT EduKit reference hardware.
 *
 * This example takes the parameters from the build configuration and establishes a connection to AWS IoT Core over MQTT.
 *
 * Some configuration is required. Visit https://edukit.workshop.aws
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "cJSON.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"

#include "core2forAWS.h"

#include "wifi.h"
#include "blink.h"
#include "ui.h"

#define STARTING_ROOMTEMPERATURE 0.0f

/* The time between each MQTT message publish in milliseconds */
#define PUBLISH_INTERVAL_MS 5000

/* The time between each sensor data gathering period (in seconds)*/
#define SENSOR_READING_AVERAGE_DELAY 4.6

/* The time between reading the sensor data once, before the next read (in seconds). 
   This should be a factor of SENSOR_READING_AVERAGE_DELAY, otherwise you risk doing sensor readings beyond 5 seconds*/
#define SENSOR_READING_DELAY 0.2

/* Specify variable for keeping track of time in vTaskDelayUntil */
TickType_t xLastWakeTime;

/* The time prefix used by the logger. */
static const char *TAG = "MAIN";

/* The FreeRTOS task handler for the blink task that can be used to control the task later */
TaskHandle_t xBlink;

/* CA Root certificate */
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");

/* Default MQTT HOST URL is pulled from the aws_iot_config.h */
char HostAddress[255] = AWS_IOT_MQTT_HOST;

/* Default MQTT port is pulled from the aws_iot_config.h */
uint32_t port = AWS_IOT_MQTT_PORT;

void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    ESP_LOGI(TAG, "Subscribe callback");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
    if (strstr(topicName, "/blink") != NULL) {
        // Get state of the FreeRTOS task, "blinkTask", using it's task handle.
        // Suspend or resume the task depending on the returned task state
        eTaskState blinkState = eTaskGetState(xBlink);
        if (blinkState == eSuspended){
            vTaskResume(xBlink);
        } else{
            vTaskSuspend(xBlink);
        }
    }
}

void disconnect_callback_handler(AWS_IoT_Client *pClient, void *data) {
    ESP_LOGW(TAG, "MQTT Disconnect");
    ui_textarea_add("Disconnected from AWS IoT Core...", NULL, 0);
    IoT_Error_t rc = FAILURE;

    if(pClient == NULL) {
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        } else {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}

/* Initialize all the variables to be used for gathering sensor data and averages */
float temperature = 0;
float gx = 0;
float gy = 0;
float gz = 0;
float ax = 0;
float ay = 0;
float az = 0;

float avg_temperature = 0;
float avg_gx = 0;
float avg_gy = 0;
float avg_gz = 0;
float avg_ax = 0;
float avg_ay = 0;
float avg_az = 0;

float Sum_Temperature = 0;
float Sum_Gyro_x = 0;
float Sum_Gyro_y = 0;
float Sum_Gyro_z = 0;
float Sum_Accel_x = 0;
float Sum_Accel_y = 0;
float Sum_Accel_z = 0;

/* This function will pull sensor data every ~0.05s and sum it. After ~4.9s, the data gets averaged, which is what can be pulled on the output */
void sensorData(float *Avg_Gyro_x, float *Avg_Gyro_y, float *Avg_Gyro_z, float *Avg_Accel_x, float *Avg_Accel_y, float *Avg_Accel_z, float *Avg_Temperature){

/* Reset all summing variables and the counter to zero */
    Sum_Temperature = 0;
    Sum_Gyro_x = 0;
    Sum_Gyro_y = 0;
    Sum_Gyro_z = 0;
    Sum_Accel_x = 0;
    Sum_Accel_y = 0;
    Sum_Accel_z = 0;
    int count_i = 0;

/* Create a basic timer to control the data collection loop, reporting averages according to SENSOR_READING_AVERAGE_DELAY */
    clock_t begin_Time = clock();

    while ( ((unsigned long) (clock() - begin_Time)/CLOCKS_PER_SEC) < SENSOR_READING_AVERAGE_DELAY){

        //printf("Start Sensor Gathering = %lf\n", (double) clock()/CLOCKS_PER_SEC);  ///////////////  debugging code
        MPU6886_GetTempData(&temperature);
        Sum_Temperature += (temperature * 1.8)  + 32 - 80;

/*Taking the square of each value so that averaging makes more sense with both positives and negatives coming in, 
  and so motion is weighted more (trying to pick up smaller motion)*/
        MPU6886_GetAccelData(&ax, &ay, &az);
        Sum_Accel_x += ax*ax;  
        Sum_Accel_y += ay*ay;
        Sum_Accel_z += az*az;

        MPU6886_GetGyroData(&gx, &gy, &gz);
        Sum_Gyro_x += gx*gx;
        Sum_Gyro_y += gy*gy;
        Sum_Gyro_z += gz*gz;

        //printf("End Sensor Gathering = %lf\n", (double) clock()/CLOCKS_PER_SEC);  //////////////  debugging code

        count_i++;

/* To keep from accumulating huge numbers in the Sum_xxx variables, the while loop below delays for SENSOR_READING_DELAY
   This is probably not the best implementation, but should work for the sleep monitor's purposes*/
        clock_t start_Time = clock();
        while ( ((unsigned long) (clock() - start_Time)/CLOCKS_PER_SEC) < SENSOR_READING_DELAY){}
    }

/* Calculate average values over the SENSOR_READING_AVERAGE_DELAY period */
    *Avg_Temperature = Sum_Temperature/count_i;
    *Avg_Accel_x = Sum_Accel_x/count_i;
    *Avg_Accel_y = Sum_Accel_y/count_i;
    *Avg_Accel_z = Sum_Accel_z/count_i;
    *Avg_Gyro_x = Sum_Gyro_x/count_i;
    *Avg_Gyro_y = Sum_Gyro_y/count_i;
    *Avg_Gyro_z = Sum_Gyro_z/count_i;

}

static void publisher(AWS_IoT_Client *client, char *base_topic, uint16_t base_topic_len){

    IoT_Publish_Message_Params paramsQOS1;
    paramsQOS1.qos = QOS1;
    paramsQOS1.isRetained = 0;

    cJSON *payload = cJSON_CreateObject();

    cJSON *temperature_value = cJSON_CreateNumber(avg_temperature);
    cJSON_AddItemToObject(payload, "temperature_value", temperature_value);

    cJSON *accel_x_value = cJSON_CreateNumber(avg_ax);
    cJSON *accel_y_value = cJSON_CreateNumber(avg_ay);
    cJSON *accel_z_value = cJSON_CreateNumber(avg_az);
    cJSON_AddItemToObject(payload, "accel_x_value", accel_x_value);
    cJSON_AddItemToObject(payload, "accel_y_value", accel_y_value);
    cJSON_AddItemToObject(payload, "accel_z_value", accel_z_value);

    cJSON *gyro_x_value = cJSON_CreateNumber(avg_gx);
    cJSON *gyro_y_value = cJSON_CreateNumber(avg_gy);
    cJSON *gyro_z_value = cJSON_CreateNumber(avg_gz);
    cJSON_AddItemToObject(payload, "gyro_x_value", gyro_x_value);
    cJSON_AddItemToObject(payload, "gyro_y_value", gyro_y_value);
    cJSON_AddItemToObject(payload, "gyro_z_value", gyro_z_value);

    const char *JSONPayload = cJSON_Print(payload);
    paramsQOS1.payload = (void*) JSONPayload;
    paramsQOS1.payloadLen = strlen(JSONPayload);

    // Publish and check if "ack" was sent from AWS IoT Core
    IoT_Error_t rc = aws_iot_mqtt_publish(client, base_topic, base_topic_len, &paramsQOS1);
    if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
        ESP_LOGW(TAG, "QOS1 publish ack not received.");
        rc = SUCCESS;
    }
}

void aws_iot_task(void *param) {
    //printf("Start iot Task = %lf\n", (double) clock()/CLOCKS_PER_SEC);  //////////////  debugging code
    IoT_Error_t rc = FAILURE;

    AWS_IoT_Client client;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false; // We enable this later below
    mqttInitParams.pHostURL = HostAddress;
    mqttInitParams.port = port;    
    mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = "#";
    mqttInitParams.pDevicePrivateKeyLocation = "#0";
    
#define CLIENT_ID_LEN (ATCA_SERIAL_NUM_SIZE * 2)
#define SUBSCRIBE_TOPIC_LEN (CLIENT_ID_LEN + 3)
#define BASE_PUBLISH_TOPIC_LEN (CLIENT_ID_LEN + 2)

    char *client_id = malloc(CLIENT_ID_LEN + 1);
    ATCA_STATUS ret = Atecc608_GetSerialString(client_id);
    if (ret != ATCA_SUCCESS)
    {
        printf("Failed to get device serial from secure element. Error: %i", ret);
        abort();
    }

    char subscribe_topic[SUBSCRIBE_TOPIC_LEN];
    char base_publish_topic[BASE_PUBLISH_TOPIC_LEN];
    snprintf(subscribe_topic, SUBSCRIBE_TOPIC_LEN, "%s/#", client_id);
    snprintf(base_publish_topic, BASE_PUBLISH_TOPIC_LEN, "%s/", client_id);

    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnect_callback_handler;
    mqttInitParams.disconnectHandlerData = NULL;

    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    /* Wait for WiFI to show as connected */
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);    

    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;

    connectParams.pClientID = client_id;
    connectParams.clientIDLen = CLIENT_ID_LEN;
    connectParams.isWillMsgPresent = false;
    ui_textarea_add("Connecting to AWS IoT Core...\n", NULL, 0);
    ESP_LOGI(TAG, "Connecting to AWS IoT Core at %s:%d", mqttInitParams.pHostURL, mqttInitParams.port);
    do {
        rc = aws_iot_mqtt_connect(&client, &connectParams);
        if(SUCCESS != rc) {
            ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } while(SUCCESS != rc);
    ui_textarea_add("Successfully connected!\n", NULL, 0);
    ESP_LOGI(TAG, "Successfully connected to AWS IoT Core!");
    /*
     * Enable Auto Reconnect functionality. Minimum and Maximum time for exponential backoff for retries.
     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
     */
    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if(SUCCESS != rc) {
        ui_textarea_add("Unable to set Auto Reconnect to true\n", NULL, 0);
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    ESP_LOGI(TAG, "Subscribing to '%s'", subscribe_topic);
    rc = aws_iot_mqtt_subscribe(&client, subscribe_topic, strlen(subscribe_topic), QOS0, iot_subscribe_callback_handler, NULL);
    if(SUCCESS != rc) {
        ui_textarea_add("Error subscribing\n", NULL, 0);
        ESP_LOGE(TAG, "Error subscribing : %d ", rc);
        abort();
    } else{
        ui_textarea_add("Subscribed to topic: %s\n\n", subscribe_topic, SUBSCRIBE_TOPIC_LEN) ;
        ESP_LOGI(TAG, "Subscribed to topic '%s'", subscribe_topic);
    }
    
    ESP_LOGI(TAG, "\n****************************************\n*  AWS client Id - %s  *\n****************************************\n\n",
             client_id);
    
    ui_textarea_add("Attempting publish to: %s\n", base_publish_topic, BASE_PUBLISH_TOPIC_LEN) ;
    while((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc)) {
        
        //Max time the yield function will wait for read messages
        rc = aws_iot_mqtt_yield(&client, 100);
        if(NETWORK_ATTEMPTING_RECONNECT == rc) {
            // If the client is attempting to reconnect we will skip the rest of the loop.
            continue;
        }

        ESP_LOGD(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));

        sensorData(&avg_gx, &avg_gy, &avg_gz, &avg_ax, &avg_ay, &avg_az, &avg_temperature);
        publisher(&client, base_publish_topic, BASE_PUBLISH_TOPIC_LEN);
        //printf("End iot Task Loop = %lf\n", (double) clock()/CLOCKS_PER_SEC);  //////////////  debugging code
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PUBLISH_INTERVAL_MS));
        //printf("Start iot Task Loop = %lf\n", (double) clock()/CLOCKS_PER_SEC);  //////////////  debugging code
    }

    ESP_LOGE(TAG, "An error occurred in the main loop.");
    abort();
}

void app_main()
{
    Core2ForAWS_Init();
    Core2ForAWS_Display_SetBrightness(25);
    
    ui_init();
    initialise_wifi();

    /* Initialize the tick time for use in vTaskDelayUntil*/
    xLastWakeTime = xTaskGetTickCount();

    xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 4096 * 2, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(&blink_task, "blink_task", 4096 * 1, NULL, 2, &xBlink, 1);
}
