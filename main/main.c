#define CONFIG_RMT_ENABLE_LEGACY_API 1
#include "driver/rmt.h"
#include "hal/rmt_types.h"
#include "hal/rmt_ll.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "soc/rmt_reg.h"
#include "soc/rmt_struct.h"
#include "soc/soc.h"


#define TRIGGER_GPIO CONFIG_INPUT_GPIO
#define RMT_TX_GPIO  CONFIG_LED_GPIO
#define RMT_CHANNEL  RMT_CHANNEL_0
#define RMT_CLK_DIV  80  // 1 µs ticks at 80 MHz APB clock

#define BIT1_HIGH_US 10
#define BIT1_LOW_US  5
#define BIT0_HIGH_US 3
#define BIT0_LOW_US  5

#define BITS_PER_SYMBOL 3

DRAM_ATTR static rmt_item32_t rmt_items[BITS_PER_SYMBOL];  // 1 pulse

// Fast ISR: start RMT TX by setting tx_start bit directly
IRAM_ATTR static void gpio_isr_handler(void* arg) {
    RMT.conf_ch[RMT_CHANNEL].conf1.tx_start = 1;
}





// Storage for all 8 patterns
DRAM_ATTR static rmt_item32_t binary_patterns[8][BITS_PER_SYMBOL];

// Generates 3-bit RMT waveform for number n (0–7)
void generate_rmt_binary_pattern(uint8_t n, rmt_item32_t* dest)
{
    for (int i = 0; i < BITS_PER_SYMBOL; i++) {
        bool bit = (n >> (BITS_PER_SYMBOL - 1 - i)) & 1;
        dest[i].level0 = 1;
        dest[i].duration0 = bit ? BIT1_HIGH_US : BIT0_HIGH_US;
        dest[i].level1 = 0;
        dest[i].duration1 = bit ? BIT1_LOW_US : BIT0_LOW_US;
    }
}

void preload_rmt_patterns()
{
    for (int i = 0; i < 8; i++) {
        generate_rmt_binary_pattern(i, binary_patterns[i]);
    }
}


//void load_rmt_pattern_to_hw(uint8_t value)
//{
//    //rmt_dev_t *rmt = &RMT;  // Pointer to RMT peripheral registers
//    volatile rmt_item32_t* rmt_mem = (volatile rmt_item32_t*)(SOC_RMT_MEM_START + RMT_CHANNEL * SOC_RMT_MEM_CHANNEL_SIZE);
//
//
//    for (int i = 0; i < BITS_PER_SYMBOL; i++) {
//        rmt->mem[RMT_CHANNEL].data32[i].val = binary_patterns[value][i].val;
//    }
//
//    // Reset read pointer using HAL function
//    rmt_ll_tx_mem_rd_rst(rmt, RMT_CHANNEL);
//}

void configure_rmt_legacy()
{
    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_CHANNEL,
        .gpio_num = RMT_TX_GPIO,
        .clk_div = RMT_CLK_DIV,
        .mem_block_num = 1,
        .tx_config = {
            .loop_en = false,
            .carrier_en = false,
            .idle_output_en = true,
            .idle_level = RMT_IDLE_LEVEL_LOW,
        }
    };
    rmt_config(&config);
    rmt_driver_install(RMT_CHANNEL, 0, 0);

    // Prepare 10 µs high + 10 µs low pulse

    // Bit 2 (MSB)
    rmt_items[0].level0 = 1;
    rmt_items[0].duration0 = 3;
    rmt_items[0].level1 = 0;
    rmt_items[0].duration1 = 3;

    // Bit 1
    rmt_items[1].level0 = 0;
    rmt_items[1].duration0 = 3;
    rmt_items[1].level1 = 0;
    rmt_items[1].duration1 = 3;

    // Bit 0 (LSB)
    rmt_items[2].level0 = 1;
    rmt_items[2].duration0 = 3;
    rmt_items[2].level1 = 0;
    rmt_items[2].duration1 = 3;

    rmt_write_items(RMT_CHANNEL, rmt_items, BITS_PER_SYMBOL, false); // Pre-load only
}

void configure_gpio_isr()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIGGER_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(TRIGGER_GPIO, gpio_isr_handler, NULL);
}

void app_main()
{
    configure_rmt_legacy();
    //preload_rmt_patterns();
    //load_rmt_pattern_to_hw(5); 
    configure_gpio_isr();
    while (true) {
        __asm__ __volatile__("nop"); // tight idle loop
        //esp_rom_delay_us(50);
    }
}
