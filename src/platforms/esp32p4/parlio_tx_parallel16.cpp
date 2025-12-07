/*
 * ESP32-P4 Parallel TX Driver using PARLIO + Direct GDMA Link API
 * 
 * This implementation uses PARLIO for GPIO/clock management but accesses
 * the internal GDMA link lists directly for precise HUB75 BCM control.
 * 
 * KEY APPROACH:
 * 1. PARLIO sets up GPIO routing and clock generation
 * 2. We access PARLIO's internal GDMA channel and link lists
 * 3. Manual DMA descriptor management for BCM timing control
 * 4. Full compatibility with ESP32-S3 HUB75 library architecture
 */

#include "sdkconfig.h"
#if defined (CONFIG_IDF_TARGET_ESP32P4)


#include "parlio_tx_parallel16.hpp"
#if SOC_PARLIO_SUPPORTED

#pragma message "Compiling for ESP32-P4 with PARLIO support"

#ifdef ARDUINO_ARCH_ESP32
   #include <Arduino.h>
#endif

#include <inttypes.h>

#include "esp_attr.h"
#include "esp_idf_version.h"
#include "hal/parlio_ll.h"
#include "hal/gdma_ll.h"
//#include "parlio_priv.h"     // Local copy of parlio private headers
//#include "gdma_link.h"       // Local copy of GDMA link API

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)		
  #include "esp_private/gpio.h"
#endif

// Callback function prototypes
static IRAM_ATTR bool parlio_tx_buffer_switched_callback(parlio_tx_unit_handle_t tx_unit, 
                                                        const parlio_tx_buffer_switched_event_data_t *event_data, 
                                                        void *user_data);

static IRAM_ATTR bool parlio_tx_done_callback(parlio_tx_unit_handle_t tx_unit, 
                                              const parlio_tx_done_event_data_t *event_data, 
                                              void *user_data);

// Static callback data
static volatile bool transmission_complete = false;






static const char* parlio_dir_to_str(parlio_dir_t dir) {
    switch (dir) {
        case PARLIO_DIR_TX: return "PARLIO_DIR_TX";
        case PARLIO_DIR_RX: return "PARLIO_DIR_RX";
        default: return "UNKNOWN_DIR";
    }
}

static const char* tx_fsm_to_str(parlio_tx_fsm_t fsm) {
    switch (fsm) {
        case PARLIO_TX_FSM_INIT:   return "PARLIO_TX_FSM_INIT";
        case PARLIO_TX_FSM_ENABLE: return "PARLIO_TX_FSM_ENABLE";
        case PARLIO_TX_FSM_RUN:    return "PARLIO_TX_FSM_RUN";
        case PARLIO_TX_FSM_WAIT:   return "PARLIO_TX_FSM_WAIT";
        default: return "UNKNOWN_FSM";
    }
}

static void dump_trans_desc(const parlio_tx_trans_desc_t* d, const char* prefix) {
    if (!d) {
        printf("%s<null>\n", prefix);
        return;
    }
    printf("%s idle_value=0x%08" PRIx32 "\n", prefix, d->idle_value);
    printf("%s payload=%p\n", prefix, d->payload);
    printf("%s payload_bits=%zu\n", prefix, d->payload_bits);
    printf("%s dma_link_idx=%d\n", prefix, d->dma_link_idx);
    printf("%s bitscrambler_program=%p\n", prefix, d->bitscrambler_program);
    printf("%s flags.loop_transmission=%d\n", prefix, (int)d->flags.loop_transmission);
}

void dump_parlio_tx_unit(const parlio_tx_unit_t* tx_unit) {
    if (!tx_unit) {
        printf("tx_unit is NULL\n");
        return;
    }

    printf("=== parlio_tx_unit_t @ %p ===\n", (const void*)tx_unit);

    // base
    printf("base.unit_id=%d\n", tx_unit->base.unit_id);
    printf("base.dir=%s (%d)\n", parlio_dir_to_str(tx_unit->base.dir), (int)tx_unit->base.dir);
    printf("base.group=%p\n", (void*)tx_unit->base.group);
    if (tx_unit->base.group) {
        printf("  group.group_id=%d\n", tx_unit->base.group->group_id);
        printf("  group.dma_align=%u\n", (unsigned)tx_unit->base.group->dma_align);
    }

    // pins & width
    printf("data_width=%zu\n", tx_unit->data_width);
    size_t cap = sizeof(tx_unit->data_gpio_nums) / sizeof(tx_unit->data_gpio_nums[0]);
    printf("data_gpio_nums (capacity=%zu):\n", cap);
    for (size_t i = 0; i < cap; ++i) {
        printf("  data_gpio_nums[%zu]=%d%s\n", i, (int)tx_unit->data_gpio_nums[i],
               (i < tx_unit->data_width) ? " (used)" : "");
    }
    printf("valid_gpio_num=%d\n", (int)tx_unit->valid_gpio_num);
    printf("clk_out_gpio_num=%d\n", (int)tx_unit->clk_out_gpio_num);
    printf("clk_in_gpio_num=%d\n", (int)tx_unit->clk_in_gpio_num);

    // handles & DMA
    printf("intr=%p\n", (void*)tx_unit->intr);
    printf("pm_lock=%p\n", (void*)tx_unit->pm_lock);
    printf("dma_chan=%p\n", (void*)tx_unit->dma_chan);
    for (int i = 0; i < PARLIO_DMA_LINK_NUM; ++i) {
        printf("dma_link[%d]=%p\n", i, (void*)tx_unit->dma_link[i]);
    }

    // memory alignment
    printf("int_mem_align=%zu\n", tx_unit->int_mem_align);
    printf("ext_mem_align=%zu\n", tx_unit->ext_mem_align);

    // optional PM lock name
    #if CONFIG_PM_ENABLE
    printf("pm_lock_name=\"%s\"\n", tx_unit->pm_lock_name);
    #endif

    // control & state
    printf("spinlock_addr=%p\n", (void*)&tx_unit->spinlock);
    printf("out_clk_freq_hz=%" PRIu32 "\n", tx_unit->out_clk_freq_hz);
    printf("clk_src=%d\n", (int)tx_unit->clk_src);
    printf("max_transfer_bits=%zu\n", tx_unit->max_transfer_bits);
    printf("queue_depth=%zu\n", tx_unit->queue_depth);
    printf("num_trans_inflight=%zu\n", tx_unit->num_trans_inflight);
    for (int q = 0; q < PARLIO_TX_QUEUE_MAX; ++q) {
        printf("trans_queues[%d]=%p\n", q, (void*)tx_unit->trans_queues[q]);
    }
    printf("cur_trans=%p\n", (void*)tx_unit->cur_trans);
    printf("idle_value_mask=0x%08" PRIx32 "\n", tx_unit->idle_value_mask);
    printf("fsm=%s (%d)\n", tx_fsm_to_str(tx_unit->fsm), (int)tx_unit->fsm);
    printf("buffer_need_switch=%s\n", tx_unit->buffer_need_switch ? "true" : "false");

    // callbacks & scrambler
    printf("on_trans_done=%p\n", (void*)tx_unit->on_trans_done);
    printf("on_buffer_switched=%p\n", (void*)tx_unit->on_buffer_switched);
    printf("user_data=%p\n", tx_unit->user_data);
    printf("bs_handle=%p\n", (void*)tx_unit->bs_handle);
    printf("bs_enable_fn=%p\n", (void*)tx_unit->bs_enable_fn);
    printf("bs_disable_fn=%p\n", (void*)tx_unit->bs_disable_fn);

    // current transaction (if any)
    if (tx_unit->cur_trans) {
        printf("-- current transaction --\n");
        dump_trans_desc(tx_unit->cur_trans, "  ");
    }

    // trans_desc_pool (assumed length = queue_depth)
    printf("-- trans_desc_pool (assumed length = queue_depth) --\n");
    for (size_t i = 0; i < tx_unit->queue_depth; ++i) {
        const parlio_tx_trans_desc_t* d = &tx_unit->trans_desc_pool[i];
        printf("  pool[%zu] @ %p\n", i, (const void*)d);
        dump_trans_desc(d, "    ");
    }

    printf("=== end of parlio_tx_unit_t ===\n");
}








// ------------------------------------------------------------------------------

void Bus_Parallel16::config(const config_t& cfg)
{
    _cfg = cfg;

        ESP_LOGI("P4_PARLIO_BCM", "Initializing ESP32-P4 PARLIO TX unit");
    
    // Fixed 16-bit data width for HUB75 compatibility
    const size_t data_width = 16;
    
    ESP_LOGI("P4_PARLIO_BCM", "Using 16-bit data width for HUB75");

    // Configure PARLIO TX unit
    parlio_tx_unit_config_t parlio_config = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = GPIO_NUM_NC,  // Use internal clock
        .input_clk_src_freq_hz = 0,  // Not used with internal clock
        .output_clk_freq_hz = _cfg.bus_freq,
        .data_width = data_width,
        .data_gpio_nums = {GPIO_NUM_NC}, // Will be set below
        .clk_out_gpio_num = (gpio_num_t)_cfg.pin_wr,
        .valid_gpio_num = GPIO_NUM_NC,  // No valid signal
        .valid_start_delay = 0,
        .valid_stop_delay = 0,
        .trans_queue_depth = 4,
        .max_transfer_size = 128*64*2,  // PARLIO will create DMA link lists internally for this size
        .dma_burst_size = 16,
        .sample_edge = _cfg.invert_pclk ? PARLIO_SAMPLE_EDGE_NEG : PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .flags = {
            .clk_gate_en = 0,
            .allow_pd = 0,
            .invert_valid_out = 0
        }
    };
    
    // Copy data pin configuration - all 16 pins for HUB75
    for (size_t i = 0; i < 16; i++) {
        parlio_config.data_gpio_nums[i] = (gpio_num_t)_cfg.pin_data[i];
    }
    
    // Create PARLIO TX unit
    esp_err_t ret = parlio_new_tx_unit(&parlio_config, &_parlio_unit);
    if (ret != ESP_OK) {
        ESP_LOGE("P4_PARLIO_BCM", "Failed to create PARLIO TX unit: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI("P4_PARLIO_BCM", "created PARLIO TX unit");

    // Setup transmit configuration
    _transmit_config.idle_value = 0x00;
    _transmit_config.bitscrambler_program = NULL;
    _transmit_config.flags.queue_nonblocking = 0;
    _transmit_config.flags.loop_transmission = 1; // Enable loop transmission for continuous operation
    
    // Register event callbacks
    parlio_tx_event_callbacks_t callbacks = {
        .on_trans_done = parlio_tx_done_callback,
        .on_buffer_switched = parlio_tx_buffer_switched_callback
    };
    
    ret = parlio_tx_unit_register_event_callbacks(_parlio_unit, &callbacks, this);
    if (ret != ESP_OK) {
        ESP_LOGE("P4_PARLIO_BCM", "Failed to register callbacks: %s", esp_err_to_name(ret));
        parlio_del_tx_unit(_parlio_unit);
        _parlio_unit = nullptr;
        return;
    }
    
    // Enable the PARLIO TX unit
    ret = parlio_tx_unit_enable(_parlio_unit);
    if (ret != ESP_OK) {
        ESP_LOGE("P4_PARLIO_BCM", "Failed to enable PARLIO TX unit: %s", esp_err_to_name(ret));
        parlio_del_tx_unit(_parlio_unit);
        _parlio_unit = nullptr;
        return;
    }
    static uint8_t data[128*64*2] = {0};
    parlio_tx_unit_transmit(_parlio_unit, data,128*64*2*8, &_transmit_config);
}

bool Bus_Parallel16::init(void)
{

    _is_transmitting = false;
    _current_buffer_id = 0;
    
    ESP_LOGI("P4_PARLIO_BCM", "PARLIO TX unit initialized successfully");
    return true;
}

void Bus_Parallel16::release(void)
{
    dma_bus_deinit();
    
    if (_parlio_unit) {
        parlio_tx_unit_disable(_parlio_unit);
        parlio_del_tx_unit(_parlio_unit);
        _parlio_unit = nullptr;
    }
    
    _dmadesc_count = 0;
    _is_transmitting = false;
}

void Bus_Parallel16::dma_bus_deinit(void)
{
    // Free GDMA link lists
    if (_gdma_link_list_a) {
        gdma_del_link_list(_gdma_link_list_a);
        _gdma_link_list_a = nullptr;
    }
    
    if (_gdma_link_list_a_loop) {
        gdma_del_link_list(_gdma_link_list_a_loop);
        _gdma_link_list_a_loop = nullptr;
    }
    
    if (_gdma_link_list_b) {
        gdma_del_link_list(_gdma_link_list_b);
        _gdma_link_list_b = nullptr;
    }
    
    if (_gdma_link_list_b_loop) {
        gdma_del_link_list(_gdma_link_list_b_loop);
        _gdma_link_list_b_loop = nullptr;
    }
    
    _dmadesc_a_idx = 0;
    _dmadesc_b_idx = 0;
    _gdma_channel = nullptr;
    _parlio_dma_link_default = nullptr;
    
    ESP_LOGI("P4_PARLIO_BCM", "DMA bus deinitialized");
}

void Bus_Parallel16::enable_double_dma_desc(void)
{
    ESP_LOGI("P4_PARLIO_BCM", "Enabled support for secondary DMA buffer.");    
    _double_dma_buffer = true;
}

bool Bus_Parallel16::allocate_dma_desc_memory(uint16_t num_descriptors)
{
    if (num_descriptors == 0) {
        ESP_LOGE("P4_PARLIO_BCM", "Number of descriptors must be greater than 0");
        return false;
    }
    
    // Free existing allocations if any
    dma_bus_deinit();
    
    ESP_LOGI("P4_PARLIO_BCM", "Allocating 4 GDMA link lists for %d descriptors each", num_descriptors);
    
    _dmadesc_count = num_descriptors;
    _max_descriptor_count = num_descriptors;
    _dmadesc_a_idx = 0;
    _dmadesc_b_idx = 0;
    
    // Access the internal GDMA channel from PARLIO unit
    if (!_parlio_unit) {
        ESP_LOGE("P4_PARLIO_BCM", "PARLIO unit not initialized");
        return false;
    }
    
    // Get PARLIO's internal GDMA channel and default DMA link (accessing private structure)
    parlio_tx_unit_t* tx_unit = (parlio_tx_unit_t*)_parlio_unit;
    _gdma_channel = tx_unit->dma_chan;
    
    // Store reference to PARLIO's default DMA link list (index 0)
    if (tx_unit->dma_link[0]) {
        _parlio_dma_link_default = tx_unit->dma_link[0];
        ESP_LOGI("P4_PARLIO_BCM", "Captured PARLIO default DMA link: %p", _parlio_dma_link_default);
    } else {
        ESP_LOGW("P4_PARLIO_BCM", "PARLIO default DMA link is NULL");
    }
    
    ESP_LOGI("P4_PARLIO_BCM", "GDMA Channel address: %p", _gdma_channel);
    //dump_parlio_tx_unit(tx_unit);

    
    // Create GDMA link lists for manual descriptor management
    gdma_link_list_config_t link_config = {
        .num_items = num_descriptors,
        .flags = {
            .items_in_ext_mem = false,
            .check_owner = false,
        }
    };
    
    // Allocate link list A (main list)
    esp_err_t ret = gdma_new_link_list(&link_config, &_gdma_link_list_a);
    if (ret != ESP_OK) {
        ESP_LOGE("P4_PARLIO_BCM", "Failed to create GDMA link list A: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Allocate link list A loop (loops back to itself by default)
    ret = gdma_new_link_list(&link_config, &_gdma_link_list_a_loop);
    if (ret != ESP_OK) {
        ESP_LOGE("P4_PARLIO_BCM", "Failed to create GDMA link list A loop: %s", esp_err_to_name(ret));
        gdma_del_link_list(_gdma_link_list_a);
        _gdma_link_list_a = nullptr;
        return false;
    }
    
    // Allocate link list B if double buffering is enabled
    if (_double_dma_buffer) {
        ret = gdma_new_link_list(&link_config, &_gdma_link_list_b);
        if (ret != ESP_OK) {
            ESP_LOGE("P4_PARLIO_BCM", "Failed to create GDMA link list B: %s", esp_err_to_name(ret));
            gdma_del_link_list(_gdma_link_list_a);
            gdma_del_link_list(_gdma_link_list_a_loop);
            _gdma_link_list_a = nullptr;
            _gdma_link_list_a_loop = nullptr;
            return false;
        }
        
        // Allocate link list B loop (loops back to itself by default)
        ret = gdma_new_link_list(&link_config, &_gdma_link_list_b_loop);
        if (ret != ESP_OK) {
            ESP_LOGE("P4_PARLIO_BCM", "Failed to create GDMA link list B loop: %s", esp_err_to_name(ret));
            gdma_del_link_list(_gdma_link_list_a);
            gdma_del_link_list(_gdma_link_list_a_loop);
            gdma_del_link_list(_gdma_link_list_b);
            _gdma_link_list_a = nullptr;
            _gdma_link_list_a_loop = nullptr;
            _gdma_link_list_b = nullptr;
            return false;
        }
    }
    
    ESP_LOGI("P4_PARLIO_BCM", "Successfully allocated 4 GDMA link lists for manual descriptor management");
    ESP_LOGI("P4_PARLIO_BCM", "Link lists: A=%p, A_loop=%p, B=%p, B_loop=%p", 
             _gdma_link_list_a, _gdma_link_list_a_loop, _gdma_link_list_b, _gdma_link_list_b_loop);
    
    return true;
}

void Bus_Parallel16::create_dma_desc_link(void *data, size_t size, bool dmadesc_b)
{
    // This function creates GDMA link items in our custom link lists
    // We populate both the main list and the loop list with the same data
    
    if (size == 0 || data == nullptr) {
        ESP_LOGW("P4_PARLIO_BCM", "Invalid data or size in create_dma_desc_link");
        return;
    }
    
    if (size > PARLIO_DMA_DESCRIPTOR_BUFFER_MAX_SIZE) {
        size = PARLIO_DMA_DESCRIPTOR_BUFFER_MAX_SIZE;
        ESP_LOGW("P4_PARLIO_BCM", "Limiting descriptor size to %d bytes", PARLIO_DMA_DESCRIPTOR_BUFFER_MAX_SIZE);
    }
    
    // Select the appropriate pair of link lists (main and loop)
    gdma_link_list_handle_t target_list = dmadesc_b ? _gdma_link_list_b : _gdma_link_list_a;
    gdma_link_list_handle_t target_list_loop = dmadesc_b ? _gdma_link_list_b_loop : _gdma_link_list_a_loop;
    uint32_t* current_idx = dmadesc_b ? &_dmadesc_b_idx : &_dmadesc_a_idx;
    
    if (target_list == nullptr || target_list_loop == nullptr) {
        ESP_LOGE("P4_PARLIO_BCM", "Target GDMA link lists not allocated");
        return;
    }
    
    if (*current_idx >= _max_descriptor_count) {
        ESP_LOGE("P4_PARLIO_BCM", "Exceeded maximum descriptor count: %d", _max_descriptor_count);
        return;
    }
    
    // Configure buffer mount for this descriptor
    gdma_buffer_mount_config_t buf_config = {
        .buffer = data,
        .length = size,
        .flags = {
            .mark_eof = 0,      // Don't mark EOF for individual descriptors
            .mark_final = 0,    // Don't mark final, we'll link manually
            .bypass_buffer_align_check = 0
        }
    };
    
    // Mount buffer to both the main link list and the loop link list
    int end_item_index;
    esp_err_t ret = gdma_link_mount_buffers(target_list, *current_idx, &buf_config, 1, &end_item_index);
    if (ret != ESP_OK) {
        ESP_LOGE("P4_PARLIO_BCM", "Failed to mount buffer to GDMA link list (main): %s", esp_err_to_name(ret));
        return;
    }
    
    ret = gdma_link_mount_buffers(target_list_loop, *current_idx, &buf_config, 1, &end_item_index);
    if (ret != ESP_OK) {
        ESP_LOGE("P4_PARLIO_BCM", "Failed to mount buffer to GDMA link list (loop): %s", esp_err_to_name(ret));
        return;
    }
    
    // If this is the last descriptor, configure the linking behavior
    if (*current_idx == _dmadesc_count - 1) {
        // Make the loop list loop back to itself
        // By default, GDMA link lists are circular, but we'll ensure proper linking during transfer start
        ESP_LOGI("P4_PARLIO_BCM", "Configured last descriptor %d for buffer %c", *current_idx, dmadesc_b ? 'B' : 'A');
    }
    
    (*current_idx)++;
    
    ESP_LOGV("P4_PARLIO_BCM", "Created DMA descriptor %d for buffer %c: ptr=%p, size=%zu", 
             *current_idx - 1, dmadesc_b ? 'B' : 'A', data, size);
}

void Bus_Parallel16::dma_transfer_start()
{
    if (!_parlio_unit || !_gdma_channel || !_gdma_link_list_a || !_gdma_link_list_a_loop) {
        ESP_LOGE("P4_PARLIO_BCM", "PARLIO unit, GDMA channel, or link lists not initialized");
        return;
    }
    
    // Strategy: Link our custom lists together to create a continuous loop
    // 1. _gdma_link_list_a (main list with BCM data)
    // 2. _gdma_link_list_a_loop (same data, continues the loop)
    // 
    // Linking: main_list -> loop_list -> main_list (creates infinite loop)
    
    // Select the active list pair based on current buffer
    gdma_link_list_handle_t active_list = _gdma_link_list_a;
    gdma_link_list_handle_t active_loop_list = _gdma_link_list_a_loop;
    
    if (_double_dma_buffer && _current_buffer_id == 1 && _gdma_link_list_b && _gdma_link_list_b_loop) {
        active_list = _gdma_link_list_b;
        active_loop_list = _gdma_link_list_b_loop;
    }
    
    // Get the head addresses - these are physical addresses for DMA
    intptr_t loop_list_head = gdma_link_get_head_addr(active_loop_list);
    intptr_t main_list_head = gdma_link_get_head_addr(active_list);
    
    ESP_LOGI("P4_PARLIO_BCM", "Main list head: 0x%08" PRIxPTR ", Loop list head: 0x%08" PRIxPTR, main_list_head, loop_list_head);
    
    // Concatenate the lists to create the loop:
    // Link the last item (-1) of main list to the first item (0) of loop list
    // Link the last item (-1) of loop list to the first item (0) of main list (circular)
    esp_err_t ret = gdma_link_concat(active_list, -1, active_loop_list, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("P4_PARLIO_BCM", "Failed to concatenate main[-1]->loop[0]: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI("P4_PARLIO_BCM", "Successfully concatenated main list to loop list");
    }
    
    // Make the loop list point back to the main list (create circular loop)
    ret = gdma_link_concat(active_loop_list, -1, active_list, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("P4_PARLIO_BCM", "Failed to concatenate loop[-1]->main[0]: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI("P4_PARLIO_BCM", "Successfully concatenated loop list back to main list (circular)");
    }
    
    // Set DMA ownership to DMA controller (not CPU)
    // This tells the DMA that it can start processing these descriptors
    // Set ownership for all items in both lists
    for (uint32_t i = 0; i < _dmadesc_count; i++) {
        ret = gdma_link_set_owner(active_list, i, GDMA_LLI_OWNER_DMA);
        if (ret != ESP_OK) {
            ESP_LOGW("P4_PARLIO_BCM", "Failed to set owner for main list[%d]: %s", i, esp_err_to_name(ret));
        }
        
        ret = gdma_link_set_owner(active_loop_list, i, GDMA_LLI_OWNER_DMA);
        if (ret != ESP_OK) {
            ESP_LOGW("P4_PARLIO_BCM", "Failed to set owner for loop list[%d]: %s", i, esp_err_to_name(ret));
        }
    }
    
    ESP_LOGI("P4_PARLIO_BCM", "Set DMA ownership for all %d descriptors in both lists", _dmadesc_count);
    
    // Link PARLIO's default DMA lists to our custom ones
    // Instead of replacing, we concatenate the PARLIO default link to our main list
    parlio_tx_unit_t* tx_unit = (parlio_tx_unit_t*)_parlio_unit;
    
    if (_parlio_dma_link_default) {
        // Link the last item of PARLIO's default list to the first item of our main list
        ret = gdma_link_concat(_parlio_dma_link_default, -1, active_list, 0);
        if (ret != ESP_OK) {
            ESP_LOGE("P4_PARLIO_BCM", "Failed to link PARLIO default to our main list: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI("P4_PARLIO_BCM", "Successfully linked PARLIO default -> our main list -> loop list (circular)");
        }
    } else {
        ESP_LOGW("P4_PARLIO_BCM", "No PARLIO default link found, cannot link to our lists");
    }
    
    _is_transmitting = true;
    ESP_LOGI("P4_PARLIO_BCM", "DMA transfer configured with link list %c", 
             (active_list == _gdma_link_list_a) ? 'A' : 'B');
    ESP_LOGI("P4_PARLIO_BCM", "PARLIO DMA is already running, no need to call gdma_start()");
}

void Bus_Parallel16::dma_transfer_stop()
{
    if (_gdma_channel && _is_transmitting) {
        esp_err_t ret = gdma_stop(_gdma_channel);
        if (ret != ESP_OK) {
            ESP_LOGE("P4_PARLIO_BCM", "Failed to stop GDMA transfer: %s", esp_err_to_name(ret));
        }
        _is_transmitting = false;
        ESP_LOGD("P4_PARLIO_BCM", "Stopped GDMA transfer");
    }
}

void Bus_Parallel16::flip_dma_output_buffer(int back_buffer_id)
{
    if (!_double_dma_buffer || !_gdma_link_list_b || !_gdma_link_list_b_loop) {
        return;
    }
    
    // Strategy for double buffering:
    // We need to re-link the new buffer pair and update PARLIO's dma_link array
    // The new buffer will be picked up on the next DMA cycle
    
    _current_buffer_id = back_buffer_id;
    
    gdma_link_list_handle_t new_list;
    gdma_link_list_handle_t new_loop_list;
    
    if (back_buffer_id == 1) {
        // Switch to buffer B
        new_list = _gdma_link_list_b;
        new_loop_list = _gdma_link_list_b_loop;
    }
    else {
        // Switch to buffer A
        new_list = _gdma_link_list_a;
        new_loop_list = _gdma_link_list_a_loop;
    }
    
    // Re-concatenate the new buffer pair to ensure proper circular linking
    esp_err_t ret = gdma_link_concat(new_list, -1, new_loop_list, 0);
    if (ret != ESP_OK) {
        ESP_LOGW("P4_PARLIO_BCM", "Failed to concatenate new main[-1]->loop[0]: %s", esp_err_to_name(ret));
    }
    
    ret = gdma_link_concat(new_loop_list, -1, new_list, 0);
    if (ret != ESP_OK) {
        ESP_LOGW("P4_PARLIO_BCM", "Failed to concatenate new loop[-1]->main[0]: %s", esp_err_to_name(ret));
    }
    
    // Set ownership to DMA for all descriptors
    for (uint32_t i = 0; i < _dmadesc_count; i++) {
        ret = gdma_link_set_owner(new_list, i, GDMA_LLI_OWNER_DMA);
        if (ret != ESP_OK) {
            ESP_LOGW("P4_PARLIO_BCM", "Failed to set owner for new main list[%d]: %s", i, esp_err_to_name(ret));
        }
        
        ret = gdma_link_set_owner(new_loop_list, i, GDMA_LLI_OWNER_DMA);
        if (ret != ESP_OK) {
            ESP_LOGW("P4_PARLIO_BCM", "Failed to set owner for new loop list[%d]: %s", i, esp_err_to_name(ret));
        }
    }
    
    // Link PARLIO's default DMA list to the new buffer
    if (_parlio_dma_link_default) {
        // Re-link the last item of PARLIO's default list to the first item of our new main list
        ret = gdma_link_concat(_parlio_dma_link_default, -1, new_list, 0);
        if (ret != ESP_OK) {
            ESP_LOGW("P4_PARLIO_BCM", "Failed to re-link PARLIO default to new buffer: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGD("P4_PARLIO_BCM", "Successfully re-linked PARLIO default to buffer %c", back_buffer_id == 1 ? 'B' : 'A');
        }
    }
    
    ESP_LOGD("P4_PARLIO_BCM", "Flipped to buffer %c", back_buffer_id == 1 ? 'B' : 'A');
    
    // Note: The actual switch will happen on the next DMA descriptor chain completion
    // PARLIO's hardware will automatically pick up the new link when the current one completes
}

// Callback implementations
static IRAM_ATTR bool parlio_tx_buffer_switched_callback(parlio_tx_unit_handle_t tx_unit, 
                                                        const parlio_tx_buffer_switched_event_data_t *event_data, 
                                                        void *user_data)
{
    Bus_Parallel16* bus = (Bus_Parallel16*)user_data;
    ESP_EARLY_LOGI("P4_PARLIO_BCM", "ARLIO buffer switched:e");
    if (bus && event_data) {
        // This callback is triggered when PARLIO internally switches buffers in loop mode
        // It indicates that the hardware has switched from one DMA buffer to another
        // This is useful for timing-sensitive applications like display refresh
        
        ESP_EARLY_LOGD("P4_PARLIO_BCM", "PARLIO buffer switched: %p -> %p", 
                      event_data->old_buffer_addr, event_data->new_buffer_addr);
        
        // You can use this callback to:
        // 1. Update display driver state
        // 2. Prepare the next frame data
        // 3. Synchronize with display refresh timing
        // 4. Signal application that it's safe to modify the old buffer
    }
    
    return false; // Don't yield from ISR
}

static IRAM_ATTR bool parlio_tx_done_callback(parlio_tx_unit_handle_t tx_unit, 
                                              const parlio_tx_done_event_data_t *event_data, 
                                              void *user_data)
{
    Bus_Parallel16* bus = (Bus_Parallel16*)user_data;
    
    // This callback is triggered when a non-loop transmission completes
    // In loop mode, this might not be called as frequently
    transmission_complete = true;
    ESP_EARLY_LOGI("P4_PARLIO_BCM", "PARLIO transmission doneA");
    if (bus) {
        // Mark transmission as complete
        // Note: In loop mode, this may indicate end of one loop iteration
        ESP_EARLY_LOGD("P4_PARLIO_BCM", "PARLIO transmission done");
    }
    
    return false; // Don't yield from ISR
}

#endif // SOC_PARLIO_SUPPORTED

#endif