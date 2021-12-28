#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_log.h>

#include "input.h"

#define ODROID_GAMEPAD_IO_X ADC1_CHANNEL_6
#define ODROID_GAMEPAD_IO_Y ADC1_CHANNEL_7
#define ODROID_GAMEPAD_IO_SELECT GPIO_NUM_27
#define ODROID_GAMEPAD_IO_START GPIO_NUM_39
#define ODROID_GAMEPAD_IO_A GPIO_NUM_32
#define ODROID_GAMEPAD_IO_B GPIO_NUM_33
#define ODROID_GAMEPAD_IO_MENU GPIO_NUM_13
#define ODROID_GAMEPAD_IO_VOLUME GPIO_NUM_0

static volatile bool input_task_is_running = false;
static volatile odroid_gamepad_state gamepad_state;
static volatile bool input_gamepad_initialized = false;
static SemaphoreHandle_t xSemaphore;


odroid_gamepad_state input_read_raw()
{
    odroid_gamepad_state state = {0};

    int joyX = adc1_get_raw(ODROID_GAMEPAD_IO_X);
    int joyY = adc1_get_raw(ODROID_GAMEPAD_IO_Y);

    if (joyX > 2048 + 1024)
    {
        state.values[ODROID_INPUT_LEFT] = 1;
        state.values[ODROID_INPUT_RIGHT] = 0;
    }
    else if (joyX > 1024)
    {
        state.values[ODROID_INPUT_LEFT] = 0;
        state.values[ODROID_INPUT_RIGHT] = 1;
    }
    else
    {
        state.values[ODROID_INPUT_LEFT] = 0;
        state.values[ODROID_INPUT_RIGHT] = 0;
    }

    if (joyY > 2048 + 1024)
    {
        state.values[ODROID_INPUT_UP] = 1;
        state.values[ODROID_INPUT_DOWN] = 0;
    }
    else if (joyY > 1024)
    {
        state.values[ODROID_INPUT_UP] = 0;
        state.values[ODROID_INPUT_DOWN] = 1;
    }
    else
    {
        state.values[ODROID_INPUT_UP] = 0;
        state.values[ODROID_INPUT_DOWN] = 0;
    }

    state.values[ODROID_INPUT_SELECT] = !(gpio_get_level(ODROID_GAMEPAD_IO_SELECT));
    state.values[ODROID_INPUT_START] = !(gpio_get_level(ODROID_GAMEPAD_IO_START));

    state.values[ODROID_INPUT_A] = !(gpio_get_level(ODROID_GAMEPAD_IO_A));
    state.values[ODROID_INPUT_B] = !(gpio_get_level(ODROID_GAMEPAD_IO_B));

    state.values[ODROID_INPUT_MENU] = !(gpio_get_level(ODROID_GAMEPAD_IO_MENU));
    state.values[ODROID_INPUT_VOLUME] = !(gpio_get_level(ODROID_GAMEPAD_IO_VOLUME));

    return state;
}

void input_read(odroid_gamepad_state* out_state)
{
    if (!input_gamepad_initialized) abort();

    xSemaphoreTake(xSemaphore, portMAX_DELAY);

    *out_state = gamepad_state;

    xSemaphoreGive(xSemaphore);
}

int input_wait_for_button_press(int ticks)
{
    odroid_gamepad_state previousState;
    input_read(&previousState);

    int timeout = xTaskGetTickCount() + ticks;
    //uint16_t btns = 0;

    while (true)
    {
		odroid_gamepad_state state;
		input_read(&state);

        for(int i = 0; i < ODROID_INPUT_MAX; i++)
        {
            if (!previousState.values[i] && state.values[i]) {
                //btns |= (1 << i);
                return i;
            }
        }

        //if (btns != 0) { we don't need multiple detection right now
        //    return btns;
        //}

        if (ticks > 0 && timeout < xTaskGetTickCount()) {
            break;
        }

        previousState = state;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    return -1;
}

static void input_task(void *arg)
{
    odroid_gamepad_state previous_gamepad_state;
    uint8_t debounce[ODROID_INPUT_MAX];

    input_task_is_running = true;

    // Initialize state
    for(int i = 0; i < ODROID_INPUT_MAX; ++i)
    {
        debounce[i] = 0xff;
    }


    while(input_task_is_running)
    {
        // Shift current values
        for(int i = 0; i < ODROID_INPUT_MAX; ++i)
		{
			debounce[i] <<= 1;
		}

        // Read hardware
        odroid_gamepad_state state = input_read_raw();

        // Debounce
        xSemaphoreTake(xSemaphore, portMAX_DELAY);

        for(int i = 0; i < ODROID_INPUT_MAX; ++i)
		{
            debounce[i] |= state.values[i] ? 1 : 0;
            uint8_t val = debounce[i] & 0x03; //0x0f;
            switch (val) {
                case 0x00:
                    gamepad_state.values[i] = 0;
                    break;

                case 0x03: //0x0f:
                    gamepad_state.values[i] = 1;
                    break;

                default:
                    // ignore
                    break;
            }
		}

        previous_gamepad_state = gamepad_state;

        xSemaphoreGive(xSemaphore);

        // delay
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    input_gamepad_initialized = false;

    vSemaphoreDelete(xSemaphore);

    // Remove the task from scheduler
    vTaskDelete(NULL);

    // Never return
    while (1) { vTaskDelay(1);}
}

void input_init()
{
    xSemaphore = xSemaphoreCreateMutex();

    if(xSemaphore == NULL)
    {
        ESP_LOGE(__func__, "xSemaphoreCreateMutex failed.");
        abort();
    }

	gpio_set_direction(ODROID_GAMEPAD_IO_SELECT, GPIO_MODE_INPUT);
	gpio_set_pull_mode(ODROID_GAMEPAD_IO_SELECT, GPIO_PULLUP_ONLY);

	gpio_set_direction(ODROID_GAMEPAD_IO_START, GPIO_MODE_INPUT);

	gpio_set_direction(ODROID_GAMEPAD_IO_A, GPIO_MODE_INPUT);
	gpio_set_pull_mode(ODROID_GAMEPAD_IO_A, GPIO_PULLUP_ONLY);

    gpio_set_direction(ODROID_GAMEPAD_IO_B, GPIO_MODE_INPUT);
	gpio_set_pull_mode(ODROID_GAMEPAD_IO_B, GPIO_PULLUP_ONLY);

	adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ODROID_GAMEPAD_IO_X, ADC_ATTEN_11db);
	adc1_config_channel_atten(ODROID_GAMEPAD_IO_Y, ADC_ATTEN_11db);

	gpio_set_direction(ODROID_GAMEPAD_IO_MENU, GPIO_MODE_INPUT);
	gpio_set_pull_mode(ODROID_GAMEPAD_IO_MENU, GPIO_PULLUP_ONLY);

	gpio_set_direction(ODROID_GAMEPAD_IO_VOLUME, GPIO_MODE_INPUT);

    input_gamepad_initialized = true;

    // Start background polling
    xTaskCreatePinnedToCore(&input_task, "input_task", 1024 * 2, NULL, 5, NULL, 1);

  	ESP_LOGI(__func__, "done.");
}
