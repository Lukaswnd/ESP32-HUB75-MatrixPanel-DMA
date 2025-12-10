#include "parlio_tx_parallel16.hpp"

#ifdef CONFIG_IDF_TARGET_ESP32P4

#pragma message "Compiling for ESP32-P4"

//First implementation might have a lot of bugs, especially on deleting and reloading

//major test setup:
// ESP32-P4 4MB Flash (USB disabled)
// 64x32 matrix (2 scanlines) P5
// esp32P4_default_pins.hpp 
// double_buffer = sometimes active, sometimes not - not much difference
// clkphase = false, latch_blanking = 1, i2sspeed = 10Mhz, colordepth = 5
// brightness = 15
// I use my custom GFX library, so neither GFX Class nor AdafruitGFX, during testing I forced it to 50hz. more would case ghosting and artifacts
// I'm to lazy to make a proper voltage supply, so lets just say - powersupply might be an issue 
// I sodered the pins to the esp32P4 devkit, but used jumer wired to conned to a HUB75 Data cable 
// which is plug into the hub 
// I'm using custom arduino build, quite close to the 3.0.1 release, based on idf-5.1 - there might be problems when updating to idf 5.2

//known bugs of this first implementation:
// - doublebuffer might shift the first view colums
// - when wriring to littlefs (maybe flash in general) flashing of the first lines, because the 
//     restart is sceduled - seems like reading ha sno effect
//     maybe the problem is the webserver which reveives the data
// - there seems to be glowing of the fist line - might be power supply issue on my side
// - BIG PROBLEM on 64x32 matrix (2 scanlines) the first 13x16 block flickers. Not the bottom half,
//     just the top half, maybe pin related? - increases by higher gfx action, never goes to zero
//     sometimer randomly stops fur multiple minutes
// - due to a single core everything (I expierienced just a view, very little artifacts with all of
//     them active at the same time: WiFi, AsyncWeserver, AsyncTCP, Filesystem, Serial, dns, mdns, sntp, custom GFX Task @ 50hz, berry-lang interpreter)
//     can cause artifacts and flashing - the problem is that the 'dma-loop' has to be retriggered
//     see gdma_on_trans_eof_callback, it may be a isr action, but still needs to reserve the core 
//     for a view clocks


// limitation of parlio interface
// PARLIO_LL_TX_MAX_BITS_PER_FRAME = (PARLIO_LL_TX_MAX_BYTES_PER_FRAME * 8)
// PARLIO_LL_TX_MAX_BYTES_PER_FRAME = 0xFFFF
// the problem are not these 2 defines. In "soc/parl_io_struct.h" (esp32-arduino-libs\esp32P4\include\soc\esp32P4\include\soc\parl_io_struct.h)
// there are only 16 bit for the number of bytes. I'm not sure, but I think this is the limiting factor. And I think
// this struct links to registers for die parlio driver, so no software change possible -> maybe software change?
// Max of 65535 bytes for matrix with 2 scan lines the following resulutions are theoretical max for pixeldepths
//                              theory                         tested
// pixeldepth 8:                              16x16             14x32            14*16*255 = 57120
// pixeldepth 7:                     16x32    32x16             28x32            28*16*127 = 56896
// pixeldepth 6:                     32x32    64x16             55x32            55*16*63  = 55440
// pixeldepth 5:             32x64   64x32   128x16            102x32           102*16*31  = 50592
// pixeldepth 4:             64x64  128x32   256x16            186x32           186*16*15  = 44640
// pixeldepth 3:            128x64  256x32   512x16            341x32           341*16*7   = 38192
// pixeldepth 2:   128x128  256x64  512x32  1024x16            682x32           682*16*2   = 32736
//                                                                              I don't get it

#ifdef ARDUINO_ARCH_ESP32
#include <Arduino.h>
#endif

#include "soc/parl_io_struct.h"
#include "soc/parl_io_reg.h"
#include "soc/parlio_periph.h"
#include "hal/parlio_hal.h"
#include "hal/parlio_ll.h"
#include "hal/parlio_types.h"

#include "esp_clk_tree.h"
#include "esp_private/periph_ctrl.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "esp_rom_gpio.h"
#include "driver/gpio.h"
#include "esp_private/esp_clk_tree_common.h"


DRAM_ATTR volatile bool previousBufferFree = true;

// End-of-DMA-transfer callback
IRAM_ATTR bool gdma_on_trans_eof_callback(gdma_channel_handle_t dma_chan,
                                          gdma_event_data_t *event_data, void *user_data)
{
    //esp_rom_delay_us(100);
    previousBufferFree = true;

    //parlio_ll_tx_reset_fifo(&PARL_IO);
//    parlio_ll_tx_reset_clock(&PARL_IO);
    
    //gdma_start(dma_chan, (intptr_t)&_dmadesc_a[0]);
    //while (parlio_ll_tx_is_ready(&PARL_IO) == false);
    //parlio_ll_tx_start(&PARL_IO, true);
    //parlio_ll_tx_enable_clock(&PARL_IO, true);

    return true;
}

// ------------------------------------------------------------------------------

void Bus_Parallel16::config(const config_t &cfg)
{
    _cfg = cfg;

}

bool Bus_Parallel16::init(void)
{
    ESP_LOGI("ESP32-P4", "Performing DMA bus init() for ESP-P4");

    // Following ESP-IDF official initialization sequence from parlio_new_tx_unit()
    printf("__A__\n");
    
    parlio_clock_source_t clk_src = (parlio_clock_source_t)PARLIO_CLK_SRC_DEFAULT;
    esp_err_t err = esp_clk_tree_enable_src((soc_module_clk_t)clk_src, true);

    printf("__B__\n");
    uint32_t periph_src_clk_hz = 0;
    esp_clk_tree_src_get_freq_hz((soc_module_clk_t)clk_src, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &periph_src_clk_hz);

    hal_utils_clk_div_t clk_div = {};
    hal_utils_clk_info_t clk_info = {
        .src_freq_hz = periph_src_clk_hz,
        .exp_freq_hz = _cfg.bus_freq,
        .max_integ = PARLIO_LL_TX_MAX_CLK_INT_DIV,
        .min_integ = 1,
        .round_opt = HAL_DIV_ROUND,
    };

    #if PARLIO_LL_TX_MAX_CLK_FRACT_DIV
        printf("__C__\n");
        clk_info.max_fract = PARLIO_LL_TX_MAX_CLK_FRACT_DIV;
        _cfg.bus_freq = hal_utils_calc_clk_div_frac_accurate(&clk_info, &clk_div);
    #else
        printf("__D__\n");
        _cfg.bus_freq = hal_utils_calc_clk_div_integer(&clk_info, &clk_div.integer);
    #endif

    
    printf("__E__\n");

    PERIPH_RCC_ATOMIC() {
        printf("__F__\n");
        // turn on the tx module clock to sync the clock divider configuration because of the CDC (Cross Domain Crossing)
        parlio_ll_tx_enable_clock(&PARL_IO, true);
        parlio_ll_tx_set_clock_source(&PARL_IO, clk_src);
        // set clock division
        parlio_ll_tx_set_clock_div(&PARL_IO, &clk_div);
    }

    // STEP 1: Enable peripheral bus clock and reset (like parlio_acquire_group_handle)
    printf("__G__\n");
    PERIPH_RCC_ATOMIC(){
        parlio_ll_enable_bus_clock(0, true);  // group_id = 0 for PARLIO
        parlio_ll_reset_register(0);
    }
    printf("__H__\n");
    
    // STEP 2: Enable clock tree source (CRITICAL - was missing!)

    if (err != ESP_OK) {
        ESP_LOGE("P4", "Failed to enable clock source: %d", err);
        return false;
    }
    printf("__I__\n");
    

    ESP_LOGI("P4", "Clock divider is %d %d %d", (int)clk_div.integer, clk_div.numerator, clk_div.denominator);
    ESP_LOGD("P4", "Resulting output clock frequency: %d Mhz", (int)(160000000L /  _cfg.bus_freq));


    // Allocate DMA channel and connect it to the LCD peripheral
    static gdma_channel_alloc_config_t dma_chan_config = {
        .sibling_chan = NULL,
        .direction = GDMA_CHANNEL_DIRECTION_TX,
        .flags = {
            .reserve_sibling = 0}};
    gdma_new_axi_channel(&dma_chan_config, &dma_chan);
    gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_PARLIO, 0));
    static gdma_strategy_config_t strategy_config = {
        .owner_check = false,
        .auto_update_desc = false};
    gdma_apply_strategy(dma_chan, &strategy_config);
    printf("__L__\n");

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
    gdma_transfer_config_t transfer_config = {
#ifdef SPIRAM_DMA_BUFFER
      .max_data_burst_size = 64,
      .access_ext_mem = true
#else
      .max_data_burst_size = 32,
      .access_ext_mem = false
#endif
    };
    gdma_config_transfer(dma_chan, &transfer_config);
#else
    gdma_transfer_ability_t ability = {
        .sram_trans_align = 32,
        .psram_trans_align = 64,
    };
    gdma_set_transfer_ability(dma_chan, &ability);
#endif

    printf("__M__\n");
    // Enable DMA transfer callback
    static gdma_tx_event_callbacks_t tx_cbs = {
        .on_trans_eof = gdma_on_trans_eof_callback};
    gdma_register_tx_event_callbacks(dma_chan, &tx_cbs, NULL);
    printf("__N__\n");


    PERIPH_RCC_ATOMIC(){
        printf("__P__\n");
        parlio_ll_tx_enable_clock(&PARL_IO, false);
    }
    printf("__P__\n");
    parlio_ll_tx_enable_clock_gating(&PARL_IO, 0);
    printf("__Q__\n");
    parlio_ll_tx_set_bus_width(&PARL_IO, 16);
    parlio_ll_tx_treat_msb_as_valid(&PARL_IO, false);
    printf("__R__\n");

    auto sample_edge = _cfg.invert_pclk ? PARLIO_SAMPLE_EDGE_NEG : PARLIO_SAMPLE_EDGE_POS;
    parlio_ll_tx_set_sample_clock_edge(&PARL_IO, sample_edge);
    printf("__S__\n");

    parlio_ll_clear_interrupt_status(&PARL_IO, PARLIO_LL_EVENT_TX_MASK);
    printf("__T__\n");


    int8_t *pins = _cfg.pin_data;
    gpio_config_t gpio_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_hal_context_t gpio_hal = {
        .dev = GPIO_HAL_GET_HW(GPIO_PORT_0)
    };

    for (int i = 0; i < 16; i++)
    {
        if (pins[i] >= 0)
        { // -1 value will CRASH the ESP32!
            gpio_conf.pin_bit_mask = BIT64(pins[i]);
            gpio_config(&gpio_conf);
            esp_rom_gpio_connect_out_signal(pins[i], parlio_periph_signals.groups[0].tx_units[0].data_sigs[i], false, false);
            gpio_hal_func_sel(&gpio_hal, GPIO_PIN_MUX_REG[pins[i]], PIN_FUNC_GPIO);
            gpio_set_drive_capability((gpio_num_t)pins[i], (gpio_drive_cap_t)3);
            printf("__U__ Configured data pin %d (GPIO %d)\n", i, pins[i]);
        }
    }
    
    printf("__V__\n");

    // Clock
    gpio_conf.pin_bit_mask = BIT64(_cfg.pin_wr);
    gpio_config(&gpio_conf);
    esp_rom_gpio_connect_out_signal(_cfg.pin_wr, parlio_periph_signals.groups[0].tx_units[0].clk_out_sig, _cfg.invert_pclk, false);
    gpio_hal_func_sel(&gpio_hal, GPIO_PIN_MUX_REG[_cfg.pin_wr], PIN_FUNC_GPIO);
    gpio_set_drive_capability((gpio_num_t)_cfg.pin_wr, (gpio_drive_cap_t)3);
    printf("__W__ Clock pin configured (GPIO %d)\n", _cfg.pin_wr);

    parlio_ll_tx_set_idle_data_value(&PARL_IO, 0);
    parlio_ll_tx_set_trans_bit_len(&PARL_IO, 0);
    printf("__X__\n");

    printf("__Y__\n");

    return true; // no return val = illegal instruction
}

void Bus_Parallel16::release(void)
{
    if (_dmadesc_a)
    {
        heap_caps_free(_dmadesc_a);
        _dmadesc_a = nullptr;
       
    }
    if(_dmadesc_b){
        heap_caps_free(_dmadesc_b);
        _dmadesc_b = nullptr;
    }   
     _dmadesc_count = 0;
}

void Bus_Parallel16::enable_double_dma_desc(void)
{
    ESP_LOGI("P4", "Enabled support for secondary DMA buffer.");
    _double_dma_buffer = true;
}

// Need this to work for double buffers etc.
bool Bus_Parallel16::allocate_dma_desc_memory(size_t len)
{
    if (_dmadesc_a)
        heap_caps_free(_dmadesc_a); // free all dma descrptios previously
    _dmadesc_count = len;

    ESP_LOGD("P4", "Allocating %d bytes memory for DMA descriptors.", (int)sizeof(HUB75_DMA_DESCRIPTOR_T) * len);

    _dmadesc_a = (HUB75_DMA_DESCRIPTOR_T *)heap_caps_malloc(sizeof(HUB75_DMA_DESCRIPTOR_T) * len, MALLOC_CAP_DMA);

    if (_dmadesc_a == nullptr)
    {
        ESP_LOGE("P4", "ERROR: Couldn't malloc _dmadesc_a. Not enough memory.");
        return false;
    }

    if (_double_dma_buffer)
    {
        _dmadesc_b = (HUB75_DMA_DESCRIPTOR_T *)heap_caps_malloc(sizeof(HUB75_DMA_DESCRIPTOR_T) * len, MALLOC_CAP_DMA);

        if (_dmadesc_b == nullptr)
        {
            ESP_LOGE("P4", "ERROR: Couldn't malloc _dmadesc_b. Not enough memory.");
            _double_dma_buffer = false;
        }
    }

    /// override static
    _dmadesc_a_idx = 0;
    _dmadesc_b_idx = 0;

    return true;
}

void Bus_Parallel16::create_dma_desc_link(void *data, size_t size, bool dmadesc_b)
{
    static constexpr size_t MAX_DMA_LEN = (4096 - 4);

    if (size > MAX_DMA_LEN)
    {
        size = MAX_DMA_LEN;
        ESP_LOGW("P4", "Creating DMA descriptor which links to payload with size greater than MAX_DMA_LEN!");
    }

    if (dmadesc_b == true)
    {

        _dmadesc_b[_dmadesc_b_idx].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
        //_dmadesc_b[_dmadesc_b_idx].dw0.suc_eof = 0;
        _dmadesc_b[_dmadesc_b_idx].dw0.suc_eof = (_dmadesc_b_idx == (_dmadesc_count - 1));
        _dmadesc_b[_dmadesc_b_idx].dw0.size = _dmadesc_b[_dmadesc_b_idx].dw0.length = size; // sizeof(data);
        _dmadesc_b[_dmadesc_b_idx].buffer = data;                                           // data;

        if (_dmadesc_b_idx == _dmadesc_count - 1)
        {
            _dmadesc_b[_dmadesc_b_idx].next = (dma_descriptor_t *)&_dmadesc_b[0];
        }
        else
        {
            _dmadesc_b[_dmadesc_b_idx].next = (dma_descriptor_t *)&_dmadesc_b[_dmadesc_b_idx + 1];
        }

        _dmadesc_b_idx++;
    }
    else
    {

        if (_dmadesc_a_idx >= _dmadesc_count)
        {
            ESP_LOGE("P4", "Attempted to create more DMA descriptors than allocated. Expecting max %u descriptors.", (unsigned int)_dmadesc_count);
            return;
        }

        _dmadesc_a[_dmadesc_a_idx].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
        //_dmadesc_a[_dmadesc_a_idx].dw0.suc_eof = 0;
        _dmadesc_a[_dmadesc_a_idx].dw0.suc_eof = (_dmadesc_a_idx == (_dmadesc_count - 1));
        _dmadesc_a[_dmadesc_a_idx].dw0.size = _dmadesc_a[_dmadesc_a_idx].dw0.length = size; // sizeof(data);
        _dmadesc_a[_dmadesc_a_idx].buffer = data;                                           // data;

        if (_dmadesc_a_idx == _dmadesc_count - 1)
        {
            _dmadesc_a[_dmadesc_a_idx].next = (dma_descriptor_t *)&_dmadesc_a[0];
        }
        else
        {
            _dmadesc_a[_dmadesc_a_idx].next = (dma_descriptor_t *)&_dmadesc_a[_dmadesc_a_idx + 1];
        }

        _dmadesc_a_idx++;
    }

} // end create_dma_desc_link

void Bus_Parallel16::dma_transfer_start()
{
    printf("__A__\n");
    PERIPH_RCC_ATOMIC() { 
        printf("__A__\n");
        parlio_ll_tx_reset_fifo(&PARL_IO);
        printf("__A__\n");
        parlio_ll_tx_reset_clock(&PARL_IO);
        
        printf("__A__\n");
        gdma_start(dma_chan, (intptr_t)&_dmadesc_a[0]);
        
        printf("__A__\n");
        while (parlio_ll_tx_is_ready(&PARL_IO) == false);

        printf("__A__\n");
        parlio_ll_tx_start(&PARL_IO, true);
        printf("__A__\n");
        parlio_ll_tx_enable_clock(&PARL_IO, true);
        printf("__A__\n");
    }

} // end

void Bus_Parallel16::dma_transfer_stop()
{

    gdma_stop(dma_chan);

} // end

void Bus_Parallel16::flip_dma_output_buffer(int back_buffer_id)
{

    // if ( _double_dma_buffer == false) return;

    if (back_buffer_id == 1) // change across to everything 'b''
    {
        _dmadesc_a[_dmadesc_count - 1].next = (dma_descriptor_t *)&_dmadesc_b[0];
        _dmadesc_b[_dmadesc_count - 1].next = (dma_descriptor_t *)&_dmadesc_b[0];
    }
    else
    {
        _dmadesc_b[_dmadesc_count - 1].next = (dma_descriptor_t *)&_dmadesc_a[0];
        _dmadesc_a[_dmadesc_count - 1].next = (dma_descriptor_t *)&_dmadesc_a[0];
    }

    previousBufferFree = false;

    while (!previousBufferFree)  ;

} // end flip

#endif
