#include "ui.h"
#include <lvgl.h>
#include <TFT_eSPI.h>
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define SIDE_SWITCH_PIN         0
#define FRONT_SWITCH_PIN        1
#define CONVERSION_FOR_AVG      8
#define NO_OF_SAMPLES           8
#define MIN_NO_OF_SAME_SAMPLES  4
#define SAMPLING_FREQ           5000
#define RIGHT_BUTTON            1000
#define MIDDLE_BUTTON           2500
#define LEFT_BUTTON             3000

#define NOTHING_PRESSED         0
#define LEFT_PRESSED            1
#define MIDDLE_PRESSED          2
#define RIGHT_PRESSED           3
#define LEFT_MIDDLE_PRESSED     4
#define LEFT_RIGHT_PRESSED      5
#define MIDDLE_RIGHT_PRESSED    6
#define ALL_PRESSED             7

#define WIFI_CONNECTED          1
#define WIFI_DISCONNECTED       0

#define CORE_0                  0
#define CORE_1                  1

#define LCD_WIDTH               320
#define LCD_HEIGHT              240
#define LCD_ROTATION            3

#define LCD_CORE                CORE_1
#define LCD_TASK_DELAY          1
#define LCD_TASK_PRIORITY       3
#define LCD_TASK_STACK_DEPTH    4096

#define SWITCH_CORE             CORE_1
#define SWITCH_TASK_DELAY       20
#define SWITCH_TASK_PRIORITY    2
#define SWITCH_TASK_STACK_DEPTH 4096

#define SERVER_CORE             CORE_0
#define SERVER_TASK_DELAY       1
#define SERVER_TASK_PRIORITY    2
#define SERVER_TASK_STACK_DEPTH 20480

TFT_eSPI tft = TFT_eSPI(LCD_WIDTH, LCD_HEIGHT);

void lcdTask(void *arg);
void switchTask(void *arg);

void dispInit(void);
static void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);

static void IRAM_ATTR adcISR(void);
static void IRAM_ATTR switchISR(void);

uint8_t adcPin[1] = {FRONT_SWITCH_PIN};
uint8_t adcPinCount = sizeof(adcPin) / sizeof(uint8_t);
bool switchIsrFlag = false;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LCD_WIDTH * LCD_HEIGHT / 10];

TaskHandle_t lcdTaskHandle = NULL, switchTaskHandle = NULL;

void setup()
{
    Serial.begin(115200);

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
}

/* Kept only to satisfy IDE requirements */
void loop(){}

void lcdTask(void *arg)
{
    dispInit();

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

void dispInit(void)
{
    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println(LVGL_Arduino);
    Serial.println("I am LVGL_Arduino");

    tft.begin();
    tft.setRotation(LCD_ROTATION);
    
    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, (LCD_WIDTH * LCD_HEIGHT / 10));
    
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