
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_system.h>

#include "driver/gpio.h"
#include <driver/spi_master.h>
#include <driver/spi_common.h>
#include "ST7735_Driver.h"

#include <esp_timer.h>

#include "esp32c3/rom/ets_sys.h"
#include <esp_console.h>

#include "driver/usb_serial_jtag.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_random.h"

enum DRAW_COMMDAND { 
    RECTANGLE,
    FILLED_RECTANGLE,
    ELLIPSE,
    LINE,
    CLEAR,
};

typedef struct coords{
    int x;
    int y;
} coords_t;

typedef struct draw_command{
    coords_t start_coord;
    coords_t end_coord;
    int color;
    uint8_t draw_type;
} draw_command_t;

const int boardsize_x = 32;
const int boardsize_y = 40;

int dirSelect = 0; 

// static portMUX_TYPE mySpinlock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t commandQueue;

static QueueHandle_t buttonQueue;

bool button_5_pressed = false;
bool button_21_pressed = false;
bool button_6_pressed = false;
bool button_7_pressed = false;

static TaskHandle_t snake_handle;
static TaskHandle_t screensave_handle;

static SemaphoreHandle_t snake_sem_handle;
static SemaphoreHandle_t screensave_sem_handle;

void IRAM_ATTR onButton(void *pvParams){

    int gpio_num = (int)(int*)pvParams;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(buttonQueue, (void*)&gpio_num, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

int r = 1,g = 1,b = 1;

//TODO: is this inefficient because args need to be copied? or doesn't make a difference
static void populate_draw_cmd(draw_command_t* cmd_p, int x0, int y0, int x1, int y1, unsigned int color){
    cmd_p->start_coord.x = x0;
    cmd_p->start_coord.y = y0;
    cmd_p->end_coord.x = x1;
    cmd_p->end_coord.y = y1;
    cmd_p->color = color;
} 

void checkButtonDeferred(void* pvParams){
    BaseType_t status;
    int gpio_num;
    static int64_t last_time = esp_timer_get_time()/1000;
    while(true){
        status = xQueueReceive(buttonQueue, &gpio_num, pdMS_TO_TICKS(1000));
        int64_t current_time = esp_timer_get_time()/1000;
        if(current_time - last_time > 150){ //deboucing
            last_time = current_time;
        }else{
            continue;
        }
        if(status == errQUEUE_EMPTY)
            continue;
        switch(gpio_num){ //in this order from top to bottom: up, down, left, right 
            case 21:
                ets_printf("button is 21\n");
                r=1-r;
                g=1-g;
                button_21_pressed = false;
                dirSelect = 1;
            break;
            case 7:
                ets_printf("button is 7\n");
                r=1-r;
                g=1-g;
                button_7_pressed = false;
                dirSelect = 0;
            break;
            case 6:
                ets_printf("button is 5\n"); 
                r=1-r;
                b=1-b;
                button_6_pressed = false;
                dirSelect = 2;
            break;
            case 5:
                ets_printf("button is 6\n");
                r=1-r;
                g=1-g;
                button_5_pressed = false;
                dirSelect = 3;
            break;
            default:
            break;

        }
    }

}

void do_display(void *pvParameter)
{
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
                case RECTANGLE:
                    DrawRect(command.start_coord.x, command.start_coord.y, command.end_coord.x, command.end_coord.y, command.color);
                    break;
                case FILLED_RECTANGLE:
                    DrawRectFilled(command.start_coord.x, command.start_coord.y, command.end_coord.x, command.end_coord.y, command.color);
                    break;
                case ELLIPSE:
                    DrawEllipse(command.start_coord.x, command.start_coord.y, command.end_coord.x, command.end_coord.y, command.color);
                    break;
                case LINE:
                    DrawLine(command.start_coord.x, command.start_coord.y, command.end_coord.x, command.end_coord.y, command.color);
                    break;
                case CLEAR:
                    clearDisplay();
                default:
                break;  
            }
            
        }
        vTaskDelay(2);
    }
}

#define BUF_SIZE (1024)

static void select_app(void *pvParams)
{
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = BUF_SIZE,
        .rx_buffer_size = BUF_SIZE,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");

    // configure temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE("usb_serial_jtag echo", "no memory for data");
        return;
    }

    xSemaphoreGive(screensave_sem_handle);
    while (true) {
        int len = usb_serial_jtag_read_bytes(data, (20), portMAX_DELAY);
        if (len) {
            data[len] = '\0';
            ets_printf("DATA:%s\n",data);
            draw_command_t command = {0};
            command.draw_type = CLEAR;
            if(strcmp((char*)data,"start") == 0){
                ets_printf("started!\n");
                xSemaphoreGive(snake_sem_handle);
                xSemaphoreTake(screensave_sem_handle, portMAX_DELAY);
                xQueueSend(commandQueue, &command, pdMS_TO_TICKS(1000));
            }else if(strcmp((char*)data, "exit") == 0){
                ets_printf("exitted!\n");
                xSemaphoreGive(screensave_sem_handle);
                xSemaphoreTake(snake_sem_handle, portMAX_DELAY);
                xQueueSend(commandQueue, &command, pdMS_TO_TICKS(1000));
            }
            usb_serial_jtag_write_bytes((const char *) data, len, portMAX_DELAY);
        }
        vTaskDelay(1000 / 5);
    }
}

void screen_saver(void* pvParameter){
    int ellipsePosX = 0;
    int ellipsePosY = 0;
    int ellipseVeloX = 1;
    int ellipseVeloY = 1;
    int ellipseSize = 5;
    draw_command_t command = {0};
    while(true){
        uint32_t code;
        if(xSemaphoreTake(screensave_sem_handle, 0) != pdTRUE){
            vTaskDelay(10);
            continue;
        }
        command.draw_type = ELLIPSE;
        populate_draw_cmd(&command, ellipsePosX, ellipsePosY, ellipsePosX + ellipseSize, ellipsePosY + ellipseSize, ColorRatio(0,0,0));
        xQueueSend(commandQueue, (void*)&command, pdMS_TO_TICKS(1000));
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
        populate_draw_cmd(&command, ellipsePosX, ellipsePosY, ellipsePosX + ellipseSize, ellipsePosY + ellipseSize, ColorRatio(r, g, b));
        xQueueSend(commandQueue, (void*)&command, pdMS_TO_TICKS(1000));
        xSemaphoreGive(screensave_sem_handle);
        vTaskDelay(10);
    }
}

void snakeGame(void *pvParameter){
    int len = 1;
    coords_t snake[100] = {0};
    int head_ind = 0;
    int tail_ind = 0;
    int foodX = esp_random() % boardsize_x;
    int foodY = esp_random() % boardsize_y;
    int direction[4][2] = {{0,1},{0,-1},{-1,0},{1,0}}; 
    
    snake[0].x = boardsize_x / 2;
    snake[0].y = boardsize_y / 2;
    coords_t cur_head = snake[0];
    coords_t cur_food = {0};
    cur_food.x = esp_random() % boardsize_x;
    cur_food.y = esp_random() % boardsize_y;

    int gridSize = D_HEIGHT / boardsize_x;

    draw_command_t command = {0};
  
    while(true){
        uint32_t code;
        if(xSemaphoreTake(snake_sem_handle, 0) != pdPASS){
            vTaskDelay(10);
            continue;
        }
        command.draw_type = FILLED_RECTANGLE;
        cur_head.x += direction[dirSelect][0];
        cur_head.y += direction[dirSelect][1];
        if(cur_head.x < 0 || cur_head.x >= boardsize_x || cur_head.y < 0 || cur_head.y >= boardsize_y){
            //game over
            len = 1;
            snake[0].x = boardsize_x / 2;
            snake[0].y = boardsize_y / 2;
            cur_head = snake[0];
            head_ind = 0;
            tail_ind = 0;
            cur_food.x = esp_random() % boardsize_x;
            cur_food.y = esp_random() % boardsize_y;
            command.draw_type = CLEAR;
            xQueueSend(commandQueue, (void*)&command, pdMS_TO_TICKS(1000));
            xSemaphoreGive(snake_sem_handle);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        //send to display   
        head_ind = (head_ind + 1) % 100;
        snake[head_ind] = cur_head;
        populate_draw_cmd(&command, cur_food.x * gridSize, cur_food.y * gridSize, 
            (cur_food.x + 1) * gridSize - 1, (cur_food.y + 1) * gridSize - 1, ColorRatio(1,0,0));

        xQueueSend(commandQueue, (void*)&command, pdMS_TO_TICKS(1000));

        populate_draw_cmd(&command, snake[head_ind].x * gridSize, snake[head_ind].y * gridSize,
        (snake[head_ind].x + 1) * gridSize - 1, (snake[head_ind].y + 1) * gridSize - 1, ColorRatio(1,1,1));

        xQueueSend(commandQueue, (void*)&command, pdMS_TO_TICKS(1000));

        //if hit food
        if(cur_head.x == cur_food.x && cur_head.y == cur_food.y){
            len++;
            cur_food.x = esp_random() % boardsize_x;
            cur_food.y = esp_random() % boardsize_y;
        }else{
            populate_draw_cmd(&command, snake[tail_ind].x * gridSize, snake[tail_ind].y * gridSize,
            (snake[tail_ind].x + 1) * gridSize - 1, (snake[tail_ind].y + 1) * gridSize - 1, ColorRatio(0,0,0));
            xQueueSend(commandQueue, (void*)&command, pdMS_TO_TICKS(1000));
            tail_ind = (tail_ind + 1) % 100;
        }
        xSemaphoreGive(snake_sem_handle);
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

    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(GPIO_NUM_21, onButton,(void*)21);
    gpio_isr_handler_add(GPIO_NUM_5, onButton,(void*)5);
    gpio_isr_handler_add(GPIO_NUM_6, onButton,(void*)6);
    gpio_isr_handler_add(GPIO_NUM_7, onButton,(void*)7);
}

extern "C" void app_main()
{       
    gpio_setup();
    esp_reset_reason_t reason = esp_reset_reason();
    printf("Reset reason: %d\n", reason);
    commandQueue = xQueueCreate(16, sizeof(draw_command_t));
    buttonQueue = xQueueCreate(8, sizeof(int));
    snake_sem_handle = xSemaphoreCreateBinary();
    screensave_sem_handle = xSemaphoreCreateBinary();
    // xSemaphoreGive(snake_sem_handle);
    // xSemaphoreGive(screensave_sem_handle);
    xTaskCreate(&do_display, "do disp", 2048,NULL,5,NULL);
    xTaskCreate(&checkButtonDeferred, "check button", 2048,NULL,6,NULL);
    xTaskCreate(&select_app, "app selector", 4096, NULL, 4, NULL);
    xTaskCreate(&screen_saver, "screen saver", 2048, NULL, 3, &screensave_handle);
    xTaskCreate(&snakeGame, "snakeGame", 4096, NULL, 3, &snake_handle);
}


