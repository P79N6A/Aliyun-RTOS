/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "iot_export.h"
#include "aliyun_config.h"
#include "i2c_master.h"
#include "esp_system.h"

extern bool got_ip_flag;

static char __product_key[PRODUCT_KEY_LEN + 1];
static char __device_name[DEVICE_NAME_LEN + 1];
static char __device_secret[DEVICE_SECRET_LEN + 1];

void obtain_time();

void event_handle(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    uintptr_t packet_id = (uintptr_t)msg->msg;
    iotx_mqtt_topic_info_pt topic_info = (iotx_mqtt_topic_info_pt)msg->msg;

    switch (msg->event_type) {
        case IOTX_MQTT_EVENT_UNDEF:
            EXAMPLE_TRACE("undefined event occur.");
            break;

        case IOTX_MQTT_EVENT_DISCONNECT:
            EXAMPLE_TRACE("MQTT disconnect.");
            break;

        case IOTX_MQTT_EVENT_RECONNECT:
            EXAMPLE_TRACE("MQTT reconnect.");
            break;

        case IOTX_MQTT_EVENT_SUBCRIBE_SUCCESS:
            EXAMPLE_TRACE("subscribe success, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_SUBCRIBE_TIMEOUT:
            EXAMPLE_TRACE("subscribe wait ack timeout, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_SUBCRIBE_NACK:
            EXAMPLE_TRACE("subscribe nack, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_UNSUBCRIBE_SUCCESS:
            EXAMPLE_TRACE("unsubscribe success, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_UNSUBCRIBE_TIMEOUT:
            EXAMPLE_TRACE("unsubscribe timeout, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_UNSUBCRIBE_NACK:
            EXAMPLE_TRACE("unsubscribe nack, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_PUBLISH_SUCCESS:
            EXAMPLE_TRACE("publish success, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_PUBLISH_TIMEOUT:
            EXAMPLE_TRACE("publish timeout, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_PUBLISH_NACK:
            EXAMPLE_TRACE("publish nack, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_PUBLISH_RECVEIVED:
            printf("topic message arrived but without any related handle");
            print_debug(topic_info->ptopic, topic_info->topic_len, "recv topic", 0x00);
            print_debug(topic_info->payload, topic_info->payload_len, "payload", 0x00);
            break;

        case IOTX_MQTT_EVENT_BUFFER_OVERFLOW:
            EXAMPLE_TRACE("buffer overflow, %s", msg->msg);
            break;

        default:
            EXAMPLE_TRACE("Should NOT arrive here.");
            break;
    }
}

static void _demo_message_arrive(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    iotx_mqtt_topic_info_pt ptopic_info = (iotx_mqtt_topic_info_pt) msg->msg;

    /* print topic name and topic message */
    EXAMPLE_TRACE("----");
    EXAMPLE_TRACE("packetId: %d", ptopic_info->packet_id);
    print_debug(ptopic_info->ptopic, ptopic_info->topic_len, "recv topic", 0x00);
    print_debug(ptopic_info->payload, ptopic_info->payload_len, "payload", 0x00);
    EXAMPLE_TRACE("----");
}

#ifndef MQTT_ID2_AUTH
int mqtt_client(void)
{
    int rc = 0, msg_len, cnt = 0;
    void *pclient;
    iotx_conn_info_pt pconn_info;
    iotx_mqtt_param_t mqtt_params;
    iotx_mqtt_topic_info_t topic_msg;
    char msg_pub[200];
    char *msg_buf = NULL, *msg_readbuf = NULL;
    int hum,temp;
    int tem[2];
    uint16 Valt;
    float Valtf;

    if (NULL == (msg_buf = (char *)HAL_Malloc(MQTT_MSGLEN))) {
        EXAMPLE_TRACE("not enough memory");
        rc = -1;
        goto do_exit;
    }

    if (NULL == (msg_readbuf = (char *)HAL_Malloc(MQTT_MSGLEN))) {
        EXAMPLE_TRACE("not enough memory");
        rc = -1;
        goto do_exit;
    }

    HAL_GetProductKey(__product_key);
    HAL_GetDeviceName(__device_name);
    HAL_GetDeviceSecret(__device_secret);

    /* Device AUTH */
    if (0 != IOT_SetupConnInfo(__product_key, __device_name, __device_secret, (void **)&pconn_info)) {
        EXAMPLE_TRACE("AUTH request failed!");
        rc = -1;
        goto do_exit;
    }

    /* Initialize MQTT parameter */
    memset(&mqtt_params, 0x0, sizeof(mqtt_params));

    mqtt_params.port = pconn_info->port;
    mqtt_params.host = pconn_info->host_name;
    mqtt_params.client_id = pconn_info->client_id;
    mqtt_params.username = pconn_info->username;
    mqtt_params.password = pconn_info->password;
    mqtt_params.pub_key = pconn_info->pub_key;

    mqtt_params.request_timeout_ms = 2000;
    mqtt_params.clean_session = 0;
    mqtt_params.keepalive_interval_ms = 60000;
    mqtt_params.pread_buf = msg_readbuf;
    mqtt_params.read_buf_size = MQTT_MSGLEN;
    mqtt_params.pwrite_buf = msg_buf;
    mqtt_params.write_buf_size = MQTT_MSGLEN;

    mqtt_params.handle_event.h_fp = event_handle;
    mqtt_params.handle_event.pcontext = NULL;


    /* Construct a MQTT client with specify parameter */
    pclient = IOT_MQTT_Construct(&mqtt_params);
    if (NULL == pclient)
    {
        EXAMPLE_TRACE("MQTT construct failed");
        rc = -1;
        goto do_exit;
    }

    /* Initialize topic information */
    memset(&topic_msg, 0x0, sizeof(iotx_mqtt_topic_info_t));
    do {
        /* Generate topic message */
        cnt++;

        if(cnt>2)
        {
	//       system_deep_sleep_set_option();
			system_deep_sleep(120000000);
        }

        Valt=system_adc_read();
        Valtf=(float)Valt*5.1875/1024;

        AM2320_GetValue(tem,tem+1);

        if((cnt==2))
        {
			msg_len = sprintf(msg_pub,"{\"id\":\"%d\",\"params\":{\"CurrentVoltage\":%.3f, \"CurrentHumidity\":%d,\"CurrentTemperature\":%d},\"method\":\"thing.event.property.post\"}", cnt,Valtf,tem[0],tem[1]);

			if (msg_len < 0)
			{
				EXAMPLE_TRACE("Error occur! Exit program");
				rc = -1;
				break;
			}

			topic_msg.payload = (void *)msg_pub;
			topic_msg.payload_len = msg_len;

			rc = IOT_MQTT_Publish(pclient, TOPIC_UP_PROPERTY, &topic_msg);
			if (rc < 0)
			{
				EXAMPLE_TRACE("error occur when publish");
				rc = -1;
				break;
			}

			EXAMPLE_TRACE("packet-id=%u, publish topic msg=%s", (uint32_t)rc, msg_pub);
        }

        /* handle the MQTT packet received from TCP or SSL connection */
//        IOT_MQTT_Yield(pclient, 200);
        HAL_SleepMs(3000);
    } while (1);

    IOT_MQTT_Destroy(&pclient);

do_exit:
    if (NULL != msg_buf)
    {
        HAL_Free(msg_buf);
    }

    if (NULL != msg_readbuf)
    {
        HAL_Free(msg_readbuf);
    }

    return rc;
}
#endif /* MQTT_ID2_AUTH */


void mqtt_task(void *pvParameter)
{
    printf("\nMQTT task started...\n");

    while (1)
    { // reconnect to tls
        while (!got_ip_flag)
        {
            vTaskDelay(TASK_CYCLE / portTICK_RATE_MS);
        }

        IOT_OpenLog("mqtt");
        IOT_SetLogLevel(IOT_LOG_EMERG);

        HAL_SetProductKey(PRODUCT_KEY);
        HAL_SetDeviceName(DEVICE_NAME);
        HAL_SetDeviceSecret(DEVICE_SECRET);

        obtain_time();
        printf("MQTT client example begin, free heap size:%d\n", system_get_free_heap_size());

#ifndef MQTT_ID2_AUTH
        mqtt_client();


#else
    	mqtt_client_secure();
#endif

    	IOT_DumpMemoryStats(IOT_LOG_DEBUG);
    	IOT_CloseLog();

        printf("MQTT client example end, free heap size:%d\n", system_get_free_heap_size());
    }
}
