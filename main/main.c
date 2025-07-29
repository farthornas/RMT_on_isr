#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

#define INPUT_GPIO CONFIG_INPUT_GPIO
#define OUTPUT_GPIO CONFIG_LED_GPIO
#define OUTPUT_GPIO_TEST CONFIG_TEST_GPIO

#define RMT_RESOLUTION_HZ 1 * 1000 * 1000
#define NUM_SLOTS 3 // Changed from 4
#define ON_DURATION_US 5  // 300ms
#define OFF_DURATION_US 1 // 100ms gap
#define ESP_INTR_FLAG_DEFAULT 0
#define NUM_RANGE 7
#define TEST 1

/* Some notes on the circuitry 
    The diodes cause a drop of about 1.35V 
    ie each diode in the bridge drop ~ 700mV 

    Bidirectional logic
    Adafruit 757 Logic Level Converter I2C




*/


/* RMT setup and generation */
rmt_channel_handle_t rmt_chan = NULL;
static rmt_encoder_handle_t copy_encoder = NULL;
uint8_t num = 0;

void init_rmt()
{
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = OUTPUT_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ, // this might be too large if 1000 HZ
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .flags.invert_out = false,        // do not invert output signal
        .flags.with_dma = false,          // do not need DMA backend
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &rmt_chan));
    ESP_ERROR_CHECK(rmt_enable(rmt_chan));
    
    rmt_copy_encoder_config_t encoder_config = { 0 };
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &copy_encoder));
    
}

void encode_number_to_bits(uint8_t number, uint8_t *bits)
{
    for (int i = 0; i < NUM_SLOTS; i++) {
        bits[i] = (number >> (NUM_SLOTS - 1 - i)) & 0x1;
    }
}

void build_rmt_symbols_from_bits(uint8_t *bits, rmt_symbol_word_t *symbols)
{
    for (int i = 0; i < NUM_SLOTS; i++) {
        if (bits[i]) {
            symbols[i].level0 = 1;
            symbols[i].duration0 = ON_DURATION_US;
        }
        else {
            symbols[i].level0 = 0;
            symbols[i]. duration0 = ON_DURATION_US;
        }

        symbols[i].level1 = 0; // always low gap after bit
        symbols[i].duration1 = OFF_DURATION_US; // small inter-bit gap
    }
}

void rmt_send_number(uint8_t number)
{
    uint8_t bits[NUM_SLOTS];
    rmt_symbol_word_t symbols[NUM_SLOTS];

    encode_number_to_bits(number, bits);
    build_rmt_symbols_from_bits(bits, symbols);
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,  // send once
    };

    ESP_ERROR_CHECK(rmt_transmit(rmt_chan, copy_encoder, symbols, sizeof(symbols), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_chan, portMAX_DELAY));
}

/* RMT setup and generation end */

/* ISR setup and event handler */

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void edge_detect_task(void* arg)
{
    uint32_t gpio_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
            rmt_send_number(num);
            num++;
            if(num > NUM_RANGE) {
                num = 0;
            }
            printf("GPIO[%"PRIu32"] intr, val: %d\n", gpio_num, gpio_get_level(gpio_num));

        }
    }
}

static void configure_isr(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;

    if (TEST){
        printf("Setting up for tests\n");
        io_conf.pin_bit_mask = OUTPUT_GPIO_TEST;
        //set as input mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        //enable pull-up mode
        io_conf.pull_up_en = 0;
        io_conf.pull_down_en = 0;

    }
    
    //configure GPIO with the given settings
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = INPUT_GPIO;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //start gpio task
    xTaskCreate(edge_detect_task, "edge_detect_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(INPUT_GPIO, gpio_isr_handler, (void*) INPUT_GPIO);
}

/* ISR setup and event handler  END */
void app_main(void)
{
    init_rmt();
    // configure_isr();
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;

    if (TEST){
        gpio_config_t out_conf = {
            .pin_bit_mask = (1ULL << OUTPUT_GPIO_TEST),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = 0,
            .pull_down_en = 0,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&out_conf);
    }
    
    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << INPUT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    gpio_config(&in_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //start gpio task
    xTaskCreate(edge_detect_task, "edge_detect_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    // Voltage on pin read needs to be >= 2V 
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(INPUT_GPIO, gpio_isr_handler, (void*) INPUT_GPIO);

    int cnt = 0;
    uint32_t gpio_num = OUTPUT_GPIO_TEST;
    while (1) {
        /* Toggle the LED state */
        printf("cnt: %d\n", cnt++);

        if (TEST){
            gpio_set_level(OUTPUT_GPIO_TEST, cnt % 2);
            printf("GPIO[%"PRIu32"] intr, val: %d\n", gpio_num, gpio_get_level(gpio_num));
        }
        printf("Sending number: %d\n", num);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}
