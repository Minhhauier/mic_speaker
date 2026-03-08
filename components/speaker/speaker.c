// #include <driver/i2s_std.h>
// #include <driver/gpio.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>

// #include "speaker.h"

// #define BCLK GPIO_NUM_27
// #define WS GPIO_NUM_14
// #define DOUT GPIO_NUM_13

// i2s_channel_handle_t i2s_tx_handle;

// i2s_chan_config_t chan_cfg;

// void i2s_init() {
//     chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
//     i2s_new_channel(&chan_cfg, &i2s_tx_handle);
//     i2s_channel_enable(i2s_tx_handle);
//     i2s_std_config_t std_cfg = {
//         .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
//         .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
//         .gpio_cfg = {
//             .mclk = I2S_GPIO_UNUSED,
//             .bclk = BCLK,
//             .ws = WS,
//             .dout = DOUT,
//             .din = I2S_GPIO_UNUSED,
//             .invert_flags = {
//                 .mclk_inv = false,
//                 .bclk_inv = false,
//                 .ws_inv = false,
//             },
//         },
//     };
//     i2s_channel_init_std_mode(i2s_tx_handle, &std_cfg);
//     i2s_channel_enable(i2s_tx_handle);
// }

// void i2s_deinit() {
//     i2s_channel_disable(i2s_tx_handle);
//     i2s_del_channel(i2s_tx_handle);
// }
// void i2s_write(const char* data, size_t size) {
//     size_t bytes_written;
//     i2s_channel_write(i2s_tx_handle, data, size, &bytes_written, portMAX_DELAY);
// }
// // void main() {
// //     i2s_init();

// //     while (1)
// //     {
// //         i2s_write("Hello, I2S!", 12);
// //         vTaskDelay(1000 / portTICK_PERIOD_MS);
// //     }
// //     i2s_deinit();
// // }