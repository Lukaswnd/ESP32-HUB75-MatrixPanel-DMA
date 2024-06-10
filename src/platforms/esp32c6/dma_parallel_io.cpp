#include "dma_parallel_io.hpp"

#ifdef CONFIG_IDF_TARGET_ESP32C6

//First implementation might have a lot of bugs, especially on deleting and reloading


#pragma message "Compiling for ESP32-C6"

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

DRAM_ATTR volatile bool previousBufferFree = true;

// End-of-DMA-transfer callback
IRAM_ATTR bool gdma_on_trans_eof_callback(gdma_channel_handle_t dma_chan,
                                          gdma_event_data_t *event_data, void *user_data)
{

    //esp_rom_delay_us(100);

    previousBufferFree = true;

    return true;
}

// ------------------------------------------------------------------------------

void Bus_Parallel16::config(const config_t &cfg)
{
    _cfg = cfg;

}

bool Bus_Parallel16::init(void)
{

    periph_module_enable(PERIPH_PARLIO_MODULE);
    periph_module_reset(PERIPH_PARLIO_MODULE);

    // Reset LCD bus
    parlio_ll_tx_reset_fifo(&PARL_IO);
    esp_rom_delay_us(1000);
    _cfg.bus_freq = 1000*100;

    parlio_ll_clock_source_t clk_src = (parlio_ll_clock_source_t)PARLIO_CLK_SRC_DEFAULT;
    uint32_t periph_src_clk_hz = 0;
    esp_clk_tree_src_get_freq_hz((soc_module_clk_t)clk_src, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &periph_src_clk_hz);
    parlio_ll_tx_set_clock_source(&PARL_IO, clk_src);
    uint32_t div = (periph_src_clk_hz + _cfg.bus_freq - 1) / _cfg.bus_freq;
    parlio_ll_tx_set_clock_div(&PARL_IO, div);
    _cfg.bus_freq = periph_src_clk_hz / div;


    ESP_LOGI("C6", "Clock divider is %d", (int)div);
    ESP_LOGD("C6", "Resulting output clock frequency: %d Mhz", (int)(160000000L /  _cfg.bus_freq));

    


    // Allocate DMA channel and connect it to the LCD peripheral
    static gdma_channel_alloc_config_t dma_chan_config = {
        .sibling_chan = NULL,
        .direction = GDMA_CHANNEL_DIRECTION_TX,
        .flags = {
            .reserve_sibling = 0}};
    gdma_new_channel(&dma_chan_config, &dma_chan);
    gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_PARLIO, 0));
    static gdma_strategy_config_t strategy_config = {
        .owner_check = false,
        .auto_update_desc = false};
    gdma_apply_strategy(dma_chan, &strategy_config);

    gdma_transfer_ability_t ability = {
        .sram_trans_align = 32,
        .psram_trans_align = 64,
    };
    gdma_set_transfer_ability(dma_chan, &ability);

    // Enable DMA transfer callback
    static gdma_tx_event_callbacks_t tx_cbs = {
        .on_trans_eof = gdma_on_trans_eof_callback};
    gdma_register_tx_event_callbacks(dma_chan, &tx_cbs, NULL);


    parlio_ll_tx_reset_clock(&PARL_IO);
    parlio_ll_tx_reset_fifo(&PARL_IO);
    parlio_ll_tx_enable_clock(&PARL_IO, false);
    parlio_ll_tx_enable_clock_gating(&PARL_IO, 0);
    parlio_ll_tx_set_bus_width(&PARL_IO, 16);
    parlio_ll_tx_treat_msb_as_valid(&PARL_IO, false);

    auto sample_edge = _cfg.invert_pclk ? PARLIO_SAMPLE_EDGE_NEG : PARLIO_SAMPLE_EDGE_POS;
    parlio_ll_tx_set_sample_clock_edge(&PARL_IO, sample_edge);

    parlio_ll_clear_interrupt_status(&PARL_IO, PARLIO_LL_EVENT_TX_MASK);


    int8_t *pins = _cfg.pin_data;
    gpio_config_t gpio_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    for (int i = 0; i < 16; i++)
    {
        if (pins[i] >= 0)
        { // -1 value will CRASH the ESP32!
            gpio_conf.pin_bit_mask = BIT64(pins[i]);
            gpio_config(&gpio_conf);
            esp_rom_gpio_connect_out_signal(pins[i], parlio_periph_signals.groups[0].tx_units[0].data_sigs[i], false, false);
            gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pins[i]], PIN_FUNC_GPIO);
            gpio_set_drive_capability((gpio_num_t)pins[i], (gpio_drive_cap_t)3);
        }
    }

    // Clock
    gpio_conf.pin_bit_mask = BIT64(_cfg.pin_wr);
    gpio_config(&gpio_conf);
    esp_rom_gpio_connect_out_signal(_cfg.pin_wr, parlio_periph_signals.groups[0].tx_units[0].clk_out_sig, _cfg.invert_pclk, false);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[_cfg.pin_wr], PIN_FUNC_GPIO);
    gpio_set_drive_capability((gpio_num_t)_cfg.pin_wr, (gpio_drive_cap_t)3);

    parlio_ll_tx_set_idle_data_value(&PARL_IO, 0);
    parlio_ll_tx_set_trans_bit_len(&PARL_IO, 0);//((1<<16) -1) * 8);

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
    ESP_LOGI("C6", "Enabled support for secondary DMA buffer.");
    _double_dma_buffer = true;
}

// Need this to work for double buffers etc.
bool Bus_Parallel16::allocate_dma_desc_memory(size_t len)
{
    if (_dmadesc_a)
        heap_caps_free(_dmadesc_a); // free all dma descrptios previously
    _dmadesc_count = len;

    ESP_LOGD("C6", "Allocating %d bytes memory for DMA descriptors.", (int)sizeof(HUB75_DMA_DESCRIPTOR_T) * len);

    _dmadesc_a = (HUB75_DMA_DESCRIPTOR_T *)heap_caps_malloc(sizeof(HUB75_DMA_DESCRIPTOR_T) * len, MALLOC_CAP_DMA);

    if (_dmadesc_a == nullptr)
    {
        ESP_LOGE("C6", "ERROR: Couldn't malloc _dmadesc_a. Not enough memory.");
        return false;
    }

    if (_double_dma_buffer)
    {
        _dmadesc_b = (HUB75_DMA_DESCRIPTOR_T *)heap_caps_malloc(sizeof(HUB75_DMA_DESCRIPTOR_T) * len, MALLOC_CAP_DMA);

        if (_dmadesc_b == nullptr)
        {
            ESP_LOGE("C6", "ERROR: Couldn't malloc _dmadesc_b. Not enough memory.");
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
        ESP_LOGW("C6", "Creating DMA descriptor which links to payload with size greater than MAX_DMA_LEN!");
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
            ESP_LOGE("C6", "Attempted to create more DMA descriptors than allocated. Expecting max %u descriptors.", (unsigned int)_dmadesc_count);
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
    parlio_ll_tx_reset_fifo(&PARL_IO);
    parlio_ll_tx_reset_clock(&PARL_IO);
    
    gdma_start(dma_chan, (intptr_t)&_dmadesc_a[0]);
    
    while (parlio_ll_tx_is_ready(&PARL_IO) == false);

    parlio_ll_tx_start(&PARL_IO, true);
    parlio_ll_tx_enable_clock(&PARL_IO, true);

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

    // current_back_buffer_id ^= 1;

    previousBufferFree = false;

    // while (i2s_parallel_is_previous_buffer_free() == false) {}
    while (!previousBufferFree)
        ;

} // end flip

#endif