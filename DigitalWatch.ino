#include "ui.h"
#include <lvgl.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <TFT_eSPI.h>
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"

#define LOCAL_SSID              "MyNet"
#define LOCAL_PASS              "9431154249"

/* HW connections for switches */
#define SIDE_SWITCH_PIN         0
#define FRONT_SWITCH_PIN        1

/* Parameters for ADC */
#define CONVERSION_FOR_AVG      8
#define NO_OF_SAMPLES           8
#define MIN_NO_OF_SAME_SAMPLES  4
#define SAMPLING_FREQ           5000

/* Front buttons related parameters */
#define NOTHING_PRESSED         0
#define LEFT_PRESSED            1
#define MIDDLE_PRESSED          2
#define RIGHT_PRESSED           3
#define LEFT_MIDDLE_PRESSED     4
#define LEFT_RIGHT_PRESSED      5
#define MIDDLE_RIGHT_PRESSED    6
#define ALL_PRESSED             7

/* WiFi Status */
#define WIFI_CONNECTED          1
#define WIFI_DISCONNECTED       0

/* Processor cores */
#define CORE_0                  0
#define CORE_1                  1

/* LCD related parameters */
#define LCD_WIDTH               320
#define LCD_HEIGHT              240
#define LCD_ROTATION            3
#define LCD_BUFFER_SIZE         (LCD_WIDTH * LCD_HEIGHT / 2)

/* Parameters for LCD Task */
#define LCD_CORE                CORE_1
#define LCD_TASK_DELAY          1
#define LCD_TASK_PRIORITY       3
#define LCD_TASK_STACK_DEPTH    4096

/* Parameters for Switch Task */
#define SWITCH_CORE             CORE_1
#define SWITCH_TASK_DELAY       20
#define SWITCH_TASK_PRIORITY    2
#define SWITCH_TASK_STACK_DEPTH 4096

/* Parameters for Server Task */
#define SERVER_CORE             CORE_0
#define SERVER_TASK_DELAY       1
#define SERVER_TASK_PRIORITY    2
#define SERVER_TASK_STACK_DEPTH 20480

/* Parameters for Housekeeping Task */
#define HK_CORE                 CORE_1
#define HK_TASK_DELAY           500
#define HK_TASK_PRIORITY        5
#define HK_TASK_STACK_DEPTH     2048

TFT_eSPI tft = TFT_eSPI(LCD_WIDTH, LCD_HEIGHT);

void lcdTask(void *arg);
void switchTask(void *arg);
void serverTask(void *arg);
void housekeepingTask(void *arg);

/* WiFi event callbacks */
void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info);
void WiFiConnLost(WiFiEvent_t event, WiFiEventInfo_t info);
void WiFiConnReestablished(WiFiEvent_t event, WiFiEventInfo_t info);

void dispInit(void);
static void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);

void dispTimeOnUART(void);

static void IRAM_ATTR adcISR(void);
static void IRAM_ATTR switchISR(void);

struct tm currTime;
bool switchIsrFlag = false;
uint8_t adcPin[1] = {FRONT_SWITCH_PIN};
const uint8_t adcPinCount = sizeof(adcPin) / sizeof(uint8_t);

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LCD_BUFFER_SIZE];

TaskHandle_t lcdTaskHandle = NULL, switchTaskHandle = NULL,
              serverTaskHandle = NULL, hkTaskHandle = NULL;

void setup()
{
    xTaskCreatePinnedToCore(lcdTask,
                            "LCD Task",
                            LCD_TASK_STACK_DEPTH,
                            NULL,
                            LCD_TASK_PRIORITY,
                            &lcdTaskHandle,
                            LCD_CORE);

    xTaskCreatePinnedToCore(switchTask,
                            "Switch Task",
                            SWITCH_TASK_STACK_DEPTH,
                            NULL,
                            SWITCH_TASK_PRIORITY,
                            &switchTaskHandle,
                            SWITCH_CORE);
    
    xTaskCreatePinnedToCore(serverTask,
                            "Server Task",
                            SERVER_TASK_STACK_DEPTH,
                            NULL,
                            SERVER_TASK_PRIORITY,
                            &serverTaskHandle,
                            SERVER_CORE);

    xTaskCreatePinnedToCore(housekeepingTask,
                            "HK Task",
                            HK_TASK_STACK_DEPTH,
                            NULL,
                            HK_TASK_PRIORITY,
                            &hkTaskHandle,
                            HK_CORE);                        
}

/* Kept only to satisfy IDE requirements */
void loop(){}

void lcdTask(void *arg)
{
    dispInit();

    //vTaskSuspend(NULL);

    while(1)
    {
        lv_timer_handler();
        vTaskDelay(LCD_TASK_DELAY / portTICK_PERIOD_MS);
    }
}

void switchTask(void *arg)
{
    uint32_t notificationValue, previousNotificationValue;
    adc_continuous_data_t * adcResult = NULL;

    if(analogContinuous(adcPin, adcPinCount, CONVERSION_FOR_AVG, SAMPLING_FREQ, adcISR))
        Serial.println("\n ADC setup done");
    else
        Serial.println("\n ADC setup failed \n");

    if(analogContinuousStart())
        Serial.println("\n ADC started \n");
    else
        Serial.println("\n ADC failed to start \n");

    pinMode(SIDE_SWITCH_PIN, INPUT_PULLUP);
    attachInterrupt(SIDE_SWITCH_PIN, switchISR, FALLING);

    while(1)
    {
        if(switchIsrFlag == true)
        {
            switchIsrFlag = false;
            tft.drawString("Button Pressed", 0, 0);
        }

        notificationValue = ulTaskNotifyTake(pdTRUE, 0);
        if((notificationValue != NOTHING_PRESSED) &&
           (notificationValue != previousNotificationValue))
        {
            previousNotificationValue = notificationValue;
            Serial.println(notificationValue);
        }
        else if(notificationValue == NOTHING_PRESSED)
            previousNotificationValue = 0;

        vTaskDelay(SWITCH_TASK_DELAY / portTICK_PERIOD_MS);
    }
}

void serverTask(void *arg)
{
    const int gmtOffsetSec = 3600 * 5.5;
    const int daylightOffsetSec = 0;

    /* Configure WiFi events */
    WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.onEvent(WiFiConnLost, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.onEvent(WiFiConnReestablished, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  
    WiFi.begin(LOCAL_SSID, LOCAL_PASS);
    while (WiFi.status() != WL_CONNECTED) 
    {
      Serial.print(".");
      vTaskDelay(250 / portTICK_PERIOD_MS);
    }
  
    if (MDNS.begin("ESP32Box")) 
    {
        Serial.println("MDNS responder started");
    }

    configTime(gmtOffsetSec, daylightOffsetSec, "pool.ntp.org");
    time_t now = time(nullptr);
  
    while(now < 24 * 3600) 
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        Serial.print(".");
        now = time(nullptr);
    }

    Serial.println("NTP synchronized");
    time(&now);

    dispTimeOnUART();

    /* Resume other tasks */
    vTaskResume(hkTaskHandle);
    //vTaskResume(lcdTaskHandle);

    vTaskSuspend(NULL);

    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void housekeepingTask(void *arg)
{
    static uint16_t seconds, hours; 
    static uint16_t tempHours = 0, tempSeconds = 0;
    static bool normalArcMode = true, secondsDone = false, hoursDone = false;
    static struct tm previousTime;

    Serial.begin(115200);

    vTaskSuspend(NULL);
    
    if(currTime.tm_min & 0x01)
    {
        normalArcMode = false;
        lv_arc_set_mode(ui_secondArc, LV_ARC_MODE_REVERSE);
    }
    else
    {
        normalArcMode = true;
        lv_arc_set_mode(ui_secondArc, LV_ARC_MODE_NORMAL);
    }

    seconds = (currTime.tm_min * 60) + currTime.tm_sec;
    hours = (currTime.tm_hour * 300) + (currTime.tm_min * 5);
    //lv_arc_set_value(ui_minuteArc, seconds);
    //lv_arc_set_value(ui_hourArc, hours);

    while(1)
    {
        getLocalTime(&currTime);

        if(tempSeconds >= seconds)
        {
            secondsDone = true;
        }
        else
        {
            tempSeconds += 10;
        }

        if(tempHours >= hours)
        {
            hoursDone = true;
        }
        else
        {
            tempHours += 10;
        }

        if(hoursDone && secondsDone)
        {
            if(currTime.tm_sec != previousTime.tm_sec)
            {
                if((currTime.tm_sec == 0) && !(currTime.tm_min & 0x01))
                {
                    normalArcMode = true;
                    lv_arc_set_mode(ui_secondArc, LV_ARC_MODE_NORMAL);
                }
                else if((currTime.tm_sec == 0) && (currTime.tm_min & 0x01))
                {
                    normalArcMode = false;
                    lv_arc_set_mode(ui_secondArc, LV_ARC_MODE_REVERSE);
                }

                if(normalArcMode == true)
                    lv_arc_set_value(ui_secondArc, currTime.tm_sec);
                else
                    lv_arc_set_value(ui_secondArc, (60 - currTime.tm_sec));
            
                previousTime = currTime;
                
                seconds = (currTime.tm_min * 60) + currTime.tm_sec;
                hours = ((currTime.tm_hour % 12) * 300) + (currTime.tm_min * 5);
            
                lv_arc_set_value(ui_hourArc, hours);
                lv_arc_set_value(ui_minuteArc, seconds);
            }
            vTaskDelay(HK_TASK_DELAY / portTICK_PERIOD_MS);
        }
        else
        {
            lv_arc_set_value(ui_hourArc, tempHours);
            lv_arc_set_value(ui_minuteArc, tempSeconds);
            vTaskDelay(5 / portTICK_PERIOD_MS);
            seconds = (currTime.tm_min * 60) + currTime.tm_sec;
            hours = (currTime.tm_hour * 300) + (currTime.tm_min * 5);
        }
    }
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void WiFiConnLost(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.println("Disconnected from WiFi access point");
    Serial.print("WiFi lost connection. Reason: ");
    Serial.println(info.wifi_sta_disconnected.reason);
    Serial.println("Trying to Reconnect");
    WiFi.begin(LOCAL_SSID, LOCAL_PASS);
}

void WiFiConnReestablished(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.println("Connected to WiFi access point");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("signal strength (RSSI):");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
}

void dispTimeOnUART(void)
{
    char timeStr[40];

    if(getLocalTime(&currTime)) 
    {
        strftime(timeStr, sizeof(timeStr), "%A, %B %d %Y %H:%M:%S", &currTime);
        Serial.println(timeStr);
    }
    else 
        Serial.println("Failed to obtain local time");
}

void dispInit(void)
{
    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println(LVGL_Arduino);
    Serial.println("I am LVGL_Arduino");

    tft.begin();
    tft.setRotation(LCD_ROTATION);
    
    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_BUFFER_SIZE);
    
    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);

    /*Change the following line to your display resolution*/
    disp_drv.hor_res  = LCD_WIDTH;
    disp_drv.ver_res  = LCD_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
    
    ui_init();
}

static void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();
    lv_disp_flush_ready(disp_drv);
}

static void IRAM_ATTR adcISR(void)
{
    uint32_t value;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint8_t index, sampleCount, sameSampleCount;
    static uint8_t buttonState[NO_OF_SAMPLES];
    adc_continuous_data_t * adcResult = NULL;
    
    analogContinuousRead(&adcResult, 0);
    value = adcResult[0].avg_read_raw;

    /* If value is more than 4000 then no
       button is pressed and hence do nothing */
    if(value > 4000)
    {
        sampleCount = 0;
        buttonState[sampleCount] = NOTHING_PRESSED;
    }
    else
    {
        sampleCount++;
        if((value > 2920) && (value < 2940))
            buttonState[sampleCount] = LEFT_PRESSED;

        else if((value > 2375) && (value < 2400))
            buttonState[sampleCount] = MIDDLE_PRESSED;
         
        else if((value > 1930) && (value < 1955))
            buttonState[sampleCount] = LEFT_MIDDLE_PRESSED;

        else if((value > 935) && (value < 960))
            buttonState[sampleCount] = RIGHT_PRESSED;

        else if((value > 850) && (value < 880))
            buttonState[sampleCount] = LEFT_RIGHT_PRESSED;
        
        else if((value > 795) && (value < 820))
            buttonState[sampleCount] = MIDDLE_RIGHT_PRESSED;
            
        else if((value > 735) && (value < 760))
            buttonState[sampleCount] = ALL_PRESSED;
        
        else
            /* Invalid reading */
            sampleCount--;
    }

    if(sampleCount == NO_OF_SAMPLES)
    {
        sampleCount = 0;
        sameSampleCount = 0;
        
        for(index = 0; index < NO_OF_SAMPLES; index++)
        {
            if(buttonState[index] == buttonState[index + 1])
            {
                sameSampleCount++;
                value = buttonState[index];
            }
            else
                sameSampleCount = 0;
            
            if(sameSampleCount == MIN_NO_OF_SAME_SAMPLES)
            {
                /* Break the for loop */
                index = NO_OF_SAMPLES;

                /* Notify the task */
                xTaskNotifyFromISR(switchTaskHandle, value, 
                eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
            }
        }
    }
}

static void IRAM_ATTR switchISR(void)
{
    switchIsrFlag = true;
}