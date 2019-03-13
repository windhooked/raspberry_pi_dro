#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include "driver/pcnt.h"

#define RELOAD_TMR      		1
#define ENC_CHANNEL_A				13		// gpio for channels of encoder simulator
#define ENC_CHANNEL_B				15
#define PCNT_PULSE_GPIO				12		// gpio for PCNT
#define PCNT_CONTROL_GPIO			14
#define DIRECTION					25		// gpio for encoder direction input

#define PCNT_H_LIM_VAL      1000
#define PCNT_L_LIM_VAL     -1000

typedef enum {
    QUAD_ENC_MODE_1 = 1,
	QUAD_ENC_MODE_2 = 2,
	QUAD_ENC_MODE_4 = 4
} quad_encoder_mode;

// jumper GPIO12 to GPIO13 (ENC_CHANNEL_A to PCNT_PULSE_GPIO)
// jumper GPIO14 to GPIO15 (ENC_CHANNEL_B to PCNT_CONTROL_GPIO)

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

volatile int cnt = 0;
void IRAM_ATTR quad_end_sim_isr(void *para) {
	int timer_idx = (int) para;

	uint32_t intr_status = TIMERG0.int_st_timers.val;
	if((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
		TIMERG0.hw_timer[timer_idx].update = 1;
		TIMERG0.int_clr_timers.t0 = 1;
		TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;

		uint32_t encA_level = 0;
		uint32_t encB_level = 0;

		switch (cnt) {
		case 0:
			break;
		case 1:
			encA_level = 1;
			break;
		case 2:
			encA_level = 1;
			encB_level = 1;
			break;
		case 3:
			encB_level = 1;
		}

		switch(gpio_get_level(DIRECTION)) {
		case 0:
			gpio_set_level(ENC_CHANNEL_A, encA_level);
			gpio_set_level(ENC_CHANNEL_B, encB_level);
			break;
		case 1:
			gpio_set_level(ENC_CHANNEL_A, encB_level);
			gpio_set_level(ENC_CHANNEL_B, encA_level);

		}
		cnt++;
		if (cnt >= 4) { cnt = 0; }
	}
}

static void IRAM_ATTR quad_enc_isr(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

static void quad_enc_gpio_init() {
	gpio_pad_select_gpio(ENC_CHANNEL_A);
	gpio_pad_select_gpio(ENC_CHANNEL_B);
	gpio_pad_select_gpio(DIRECTION);
	gpio_set_direction(ENC_CHANNEL_A,GPIO_MODE_OUTPUT);
	gpio_set_direction(ENC_CHANNEL_B,GPIO_MODE_OUTPUT);
	gpio_set_direction(DIRECTION,GPIO_MODE_INPUT);
	gpio_set_pull_mode(DIRECTION, GPIO_PULLDOWN_ONLY);
}

static void quad_enc_sim_timer_init(int timer_idx, bool auto_reload, uint32_t frequency, timer_count_dir_t direction ) {
    timer_config_t config;
    config.alarm_en = TIMER_ALARM_EN;
    config.auto_reload = auto_reload;
    config.counter_dir = direction;
    config.divider = 2;
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_en = TIMER_PAUSE;

    timer_init(TIMER_GROUP_0, timer_idx, &config);
    timer_pause(TIMER_GROUP_0, timer_idx);
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, 40000000 / 4 / frequency);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, quad_end_sim_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

static void quadrature_encoder_counter_init(quad_encoder_mode enc_mode) {
	 pcnt_config_t pcnt_config = {
		        .pulse_gpio_num = PCNT_PULSE_GPIO,
		        .ctrl_gpio_num = PCNT_CONTROL_GPIO,
		        .channel = PCNT_CHANNEL_0,
		        .unit = PCNT_UNIT_0,
		        .pos_mode = PCNT_COUNT_INC,   			// Count up on the positive edge
		        .neg_mode = PCNT_COUNT_DIS,   			// Keep the counter value on the negative edge
		        .lctrl_mode = PCNT_MODE_KEEP, 			// Reverse counting direction if low
		        .hctrl_mode = PCNT_MODE_REVERSE,    	// Keep the primary counter mode if high
		        .counter_h_lim = PCNT_H_LIM_VAL,
		        .counter_l_lim = PCNT_L_LIM_VAL,
		    };

	switch (enc_mode) {
	case QUAD_ENC_MODE_1:
		 break;
	case QUAD_ENC_MODE_2:
		 pcnt_config.neg_mode = PCNT_COUNT_DEC;
		 break;
	case QUAD_ENC_MODE_4:
		// Doesn't appear to be possible to handle 4X mode with the PCNT. THis mode requires the count to increment when the CONTROL input changes.
		break;
	}

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(quad_enc_isr, NULL, 0, NULL);
    pcnt_intr_enable(PCNT_UNIT_0);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT_0);
}

void app_main()
{
	uint32_t frequency = 10;
	quad_enc_gpio_init();
    quad_enc_sim_timer_init(TIMER_0, RELOAD_TMR, frequency, TIMER_COUNT_UP);

    /* Initialize PCNT event queue and PCNT functions */
       pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
       quadrature_encoder_counter_init(QUAD_ENC_MODE_2);

       int16_t count = 0;
       pcnt_evt_t evt;
       portBASE_TYPE res;
       while (1) {
           /* Wait for the event information passed from PCNT's interrupt handler.
            * Once received, decode the event type and print it on the serial monitor.
            */
           res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
           if (res == pdTRUE) {
               pcnt_get_counter_value(PCNT_UNIT_0, &count);
               printf("Event PCNT unit[%d]; cnt: %d\n", evt.unit, count);
               if (evt.status & PCNT_STATUS_THRES1_M) {
                   printf("THRES1 EVT\n");
               }
               if (evt.status & PCNT_STATUS_THRES0_M) {
                   printf("THRES0 EVT\n");
               }
               if (evt.status & PCNT_STATUS_L_LIM_M) {
                   printf("L_LIM EVT\n");
               }
               if (evt.status & PCNT_STATUS_H_LIM_M) {
                   printf("H_LIM EVT\n");
               }
               if (evt.status & PCNT_STATUS_ZERO_M) {
                   printf("ZERO EVT\n");
               }
           } else {
               pcnt_get_counter_value(PCNT_UNIT_0, &count);
               printf("Current counter value :%d\n", count);
           }
       }
}

