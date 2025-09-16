
#include <stdio.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_system.h>

#include "driver/gpio.h"
#include <driver/uart.h>
#include <driver/spi_master.h>
#include <driver/spi_common.h>
#include "ST7735_Driver.h"

#include <esp_timer.h>


#include "esp32c3/rom/ets_sys.h"
#include <esp_console.h>

// #include "FreeRTOS.h"
#include "driver/usb_serial_jtag.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_random.h"


#define BLINK_GPIO 9

typedef struct coords{
    int x;
    int y;
} coords_t;

typedef struct draw_command{
    uint8_t draw_type;
    coords_t start_coord;
    coords_t end_coord;
    int color;
} draw_command_t;

const int boardSize = 32;

int dirSelect = 0; 

// static portMUX_TYPE mySpinlock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t commandQueue;

static SemaphoreHandle_t buttonEvt;

bool button_5_pressed = false;
bool button_21_pressed = false;
bool button_6_pressed = false;
bool button_7_pressed = false;

int64_t last_time = esp_timer_get_time()/1000;

void IRAM_ATTR onButton(void *pvParams){

    *(bool*)pvParams = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(buttonEvt, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

int r=1,g=1,b=1;

void checkButtonDeferred(void* pvParams){
    while(true){
        xSemaphoreTake(buttonEvt, portMAX_DELAY);
        int64_t current_time = esp_timer_get_time()/1000;
        if(current_time - last_time > 150){
            //this means that the button handling function is on
            ets_printf("Hello");
            last_time = current_time;
        }else{
            continue;
        }
        if(button_21_pressed){ //up
            printf("button is 21\n");
            r=1-r;
            g=1-g;
            button_21_pressed = false;
            dirSelect = 1;
        }
        if(button_7_pressed){ //down
            printf("button is 7\n");
            r=1-r;
            g=1-g;
            button_7_pressed = false;
            dirSelect = 0;
        }
        if(button_6_pressed){ //left
            printf("button is 5\n"); 
            r=1-r;
            b=1-b;
            button_6_pressed = false;
            dirSelect = 2;
        }
        if(button_5_pressed){ //right
            printf("button is 6\n");
            r=1-r;
            g=1-g;
            button_5_pressed = false;
            dirSelect = 3;
        }
    }

}

void do_display(void *pvParameter)
{
    // gpio_config(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    spi_device_handle_t tempSPI;
    DisplayInit(&tempSPI);
    DrawLine(64,0,64,159,ColorRatio(0.2, 0.3, 0.4));
    DrawLine(0,80,127,80,ColorRatio(0.2, 0.3, 0.4));
    DrawEllipse(0,0,127,159,ColorRatio(1, 0, 0));

    WriteText("DEMO", sizeof("DEMO")/sizeof(char), 40, 40, ColorRatio(1,1,1), ColorRatio(0,0,0));
    WriteText("PROGRAM", sizeof("PROGRAM")/sizeof(char), 40, 60, ColorRatio(1,1,1), ColorRatio(0,0,0));
    vTaskDelay(pdMS_TO_TICKS(1000));
    clearDisplay();
    draw_command_t command;
    
    while(1) {
        if(xQueueReceive(commandQueue, (void*)&command, pdMS_TO_TICKS(1000))){
            switch(command.draw_type){
                case 0:
                    DrawRect(command.start_coord.x, command.start_coord.y, command.end_coord.x, command.end_coord.y, command.color);
                    break;
                case 1:
                    DrawEllipse(command.start_coord.x, command.start_coord.y, command.end_coord.x, command.end_coord.y, command.color);
                    break;
                default:
                break;  
            }
            
        }
        vTaskDelay(2);
    }
}

#define BUF_SIZE (1024)

static void echo_task(void *arg)
{
    // Configure USB SERIAL JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = BUF_SIZE,
        .rx_buffer_size = BUF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE("usb_serial_jtag echo", "no memory for data");
        return;
    }

    while (1) {

        int len = usb_serial_jtag_read_bytes(data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);

        // Write data back to the USB SERIAL JTAG
        if (len) {
            data[len] = '\0';
            usb_serial_jtag_write_bytes((const char *) data, len, 20 / portTICK_PERIOD_MS);
            // ESP_LOG_BUFFER_HEXDUMP("Recv str: ", data, len, ESP_LOG_INFO);
        }
        vTaskDelay(1000 / 5);
    }
}

void splash_screen(void* pvParameter){
    int ellipsePosX = 0;
    int ellipsePosY = 0;
    int ellipseVeloX = 1;
    int ellipseVeloY = 1;
    int ellipseSize = 5;
    while(true){
        DrawEllipse(ellipsePosX,ellipsePosY,ellipsePosX + ellipseSize,ellipsePosY + ellipseSize,ColorRatio(0, 0, 0));
        if(ellipsePosX < 0 || ellipsePosX > max_width - ellipseSize){
            ellipseVeloX *= -1;
            ellipsePosX += ellipseVeloX;
        }
        if(ellipsePosY < 0 || ellipsePosY > max_height - ellipseSize){
            ellipseVeloY *= -1;
            ellipsePosY+= ellipseVeloY;
        }
        ellipsePosX += ellipseVeloX;
        ellipsePosY+= ellipseVeloY;
        vTaskDelay(2);
        DrawEllipse(ellipsePosX,ellipsePosY,ellipsePosX + ellipseSize,ellipsePosY + ellipseSize,ColorRatio(r, g, b));
    }
}

void snakeGame(void *pvParameter){
    int len = 1;
    coords_t snake[100] = {0};
    int head_ind = 0;
    int tail_ind = 0;
    int foodX = esp_random() % boardSize;
    int foodY = esp_random() % boardSize;
    int direction[4][2] = {{0,1},{0,-1},{-1,0},{1,0}}; 
    
    snake[0].x = boardSize / 2;
    snake[0].y = snake[0].x;
    coords_t cur_head = snake[0];
    coords_t cur_food = {0};
    cur_food.x = esp_random() % boardSize;
    cur_food.y = esp_random() % boardSize;

    int gridSize = 128 / boardSize;

    while(true){
        cur_head.x += direction[dirSelect][0];
        cur_head.y += direction[dirSelect][1];
        if(cur_head.x < 0 || cur_head.x >= boardSize || cur_head.y < 0 || cur_head.y >= boardSize){
            //game over
            len = 1;
            snake[0].x = boardSize / 2;
            snake[0].y = boardSize / 2;
            head_ind = 0;
            cur_food.x = esp_random() % boardSize;
            cur_food.y = esp_random() % boardSize;
            continue;
        }
        //send to display   
        draw_command_t command = {0};
        head_ind = (head_ind + 1) % 100;
        snake[head_ind] = cur_head;
        command.start_coord.x = cur_food.x * gridSize;
        command.start_coord.y = cur_food.y * gridSize;
        command.end_coord.x = command.start_coord.x + gridSize - 1;
        command.end_coord.y = command.start_coord.y + gridSize - 1;

        command.color = ColorRatio(1,0,0);
        xQueueSend(commandQueue, (void*)&command, pdMS_TO_TICKS(1000));

        command.start_coord.x = snake[head_ind].x * gridSize;
        command.start_coord.y = snake[head_ind].y * gridSize;
        command.end_coord.x = command.start_coord.x + gridSize - 1;
        command.end_coord.y = command.start_coord.y + gridSize - 1;

        command.color = ColorRatio(1,1,1);
        xQueueSend(commandQueue, (void*)&command, pdMS_TO_TICKS(1000));

        //if hit food
        if(cur_head.x == cur_food.x && cur_head.y == cur_food.y){
            len++;
            cur_food.x = esp_random() % boardSize;
            cur_food.y = esp_random() % boardSize;
        }else{
            command.start_coord.x = snake[tail_ind].x * gridSize;
            command.start_coord.y = snake[tail_ind].y * gridSize;
            command.end_coord.x = command.start_coord.x + gridSize - 1;
            command.end_coord.y = command.start_coord.y + gridSize - 1;
            command.color = ColorRatio(0,0,0);
            xQueueSend(commandQueue, (void*)&command, pdMS_TO_TICKS(1000));
            tail_ind = (tail_ind + 1) % 100;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void gpio_setup(){

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 21) | (1ULL << 5) | (1ULL << 6) | (1ULL << 7),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };

    buttonEvt = xSemaphoreCreateBinary();
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(GPIO_NUM_21, onButton,(void*)&button_21_pressed);
    gpio_isr_handler_add(GPIO_NUM_5, onButton,(void*)&button_5_pressed);
    gpio_isr_handler_add(GPIO_NUM_6, onButton,(void*)&button_6_pressed);
    gpio_isr_handler_add(GPIO_NUM_7, onButton,(void*)&button_7_pressed);
}

extern "C" void app_main()
{       

    gpio_setup();
    esp_reset_reason_t reason = esp_reset_reason();
    printf("Reset reason: %d\n", reason);
    commandQueue = xQueueCreate(16, sizeof(draw_command_t));
    xTaskCreate(&do_display, "do disp", 2048,NULL,5,NULL);
    xTaskCreate(&checkButtonDeferred, "check button", 2048,NULL,6,NULL);
    xTaskCreate(echo_task, "USB SERIAL JTAG_echo_task", 4096, NULL, 3, NULL);
    xTaskCreate(snakeGame, "snakeGame", 4096, NULL, 3, NULL);
    
}


