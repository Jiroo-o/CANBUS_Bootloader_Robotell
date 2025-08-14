/* STM32F103C8T6 CAN Bootloader — reliable batch ACK/NACK
 * Layout: BL 16KB @0x08000000, APP @0x08004000
 * Frames:
 *   START 0x7FF: ['S','T','A','R', size_le_4B]
 *   DATA  0x100: [idx_lo, idx_hi] + payload(≤6B)  // total 8B
 *   END   0x7FE: ['E','N','D', 0, crc32_le_4B]
 * MCU -> PC:
 *   ACK   0x101: [0xAA]
 *   READY 0x101: [0xAC]
 *   NACK  0x102: [idx_lo, idx_hi]
 */

#include "main.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* ====== Config ====== */
#define BOOTLOADER_SIZE_KB   16u
#define APP_BASE_ADDR        0x08004000UL
#define MAX_APP_PAGES        48

/* CAN IDs */
#define CAN_ID_START         0x7FF
#define CAN_ID_DATA          0x100
#define CAN_ID_END           0x7FE
#define CAN_ID_ACK           0x101
#define CAN_ID_NACK          0x102

/* Flow control */
#define BATCH_SIZE           8        // ตรงกับฝั่ง PC (ดีบัก 4 จะนิ่งสุด)
#define RX_BUF_SIZE          256
#define DEBUG_ACK_EVERY_FRAME 0       // 1 = ส่ง ACK ทุกเฟรม (ทดสอบ flow)

#define MIN(a,b) ((a)<(b)?(a):(b))

/* LED PC13 */
#define LED_PORT GPIOC
#define LED_PIN  GPIO_PIN_13
#define LED_ON()  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
#define LED_TOGGLE() HAL_GPIO_TogglePin(LED_PORT, LED_PIN)

/* Handles */
CAN_HandleTypeDef hcan;
UART_HandleTypeDef huart1;

/* ===== State ===== */
typedef enum {
	LED_MODE_OFF = 0, LED_MODE_STANDBY, LED_MODE_RX, LED_MODE_BUSY
} led_mode_t;
static volatile led_mode_t led_mode = LED_MODE_STANDBY;
static uint32_t led_last_toggle = 0, led_period_ms = 500, rx_mode_until = 0;

static volatile uint8_t firmwareUpdating = 0;
static volatile uint32_t flashAddress = APP_BASE_ADDR;
static uint32_t expected_size = 0, expected_crc = 0, bytes_written = 0;
static uint32_t current_crc = 0xFFFFFFFF;
static uint32_t expected_frame_index = 0;
static uint32_t frame_count = 0;
static uint8_t waiting_resend = 0;

/* flash accumulator: program 32-bit words */
static uint8_t wbuf[4];
static uint8_t wlen = 0;

/* ===== ring buffer for CAN RX ===== */
typedef struct {
	CAN_RxHeaderTypeDef hdr;
	uint8_t data[8];
} can_frame_t;
static volatile can_frame_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0;

static inline int rb_empty(void) {
	return rx_head == rx_tail;
}
static inline int rb_full(void) {
	return ((rx_head + 1) % RX_BUF_SIZE) == rx_tail;
}
static void rb_push(const CAN_RxHeaderTypeDef *hdr, const uint8_t *data) {
	if (!rb_full()) {
		rx_buf[rx_head].hdr = *hdr;
		memcpy((void*) rx_buf[rx_head].data, data, hdr->DLC);
		rx_head = (rx_head + 1) % RX_BUF_SIZE;
	}
}
static int rb_pop(can_frame_t *out) {
    if (rb_empty()) {
        return 0;
    }
    *out = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    return 1;
}

/* ===== Protos ===== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void CAN_ConfigFilters(void);

/* utils */
static void debug_printf(const char *fmt, ...) {
	char buf[128];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);
}
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
	for (size_t i = 0; i < len; i++) {
		uint32_t c = (crc ^ data[i]) & 0xFFu;
		for (int k = 0; k < 8; k++)
			c = (c >> 1) ^ (0xEDB88320u & (-(int) (c & 1)));
		crc = (crc >> 8) ^ c;
	}
	return crc;
}

/* LED */
static void led_init(void) {
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef G = { 0 };
	G.Pin = LED_PIN;
	G.Mode = GPIO_MODE_OUTPUT_PP;
	G.Pull = GPIO_NOPULL;
	G.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_PORT, &G);
	LED_OFF();
}
static void led_set_mode(led_mode_t m) {
	led_mode = m;
	switch (m) {
	case LED_MODE_OFF:
		LED_OFF();
		break;
	case LED_MODE_STANDBY:
		led_period_ms = 500;
		led_last_toggle = HAL_GetTick();
		LED_OFF();
		break;
	case LED_MODE_RX:
		led_period_ms = 100;
		led_last_toggle = HAL_GetTick();
		rx_mode_until = HAL_GetTick() + 300;
		break;
	case LED_MODE_BUSY:
		LED_ON();
		break;
	}
}
static void led_task(void) {
	if (led_mode == LED_MODE_RX
			&& (int32_t) (HAL_GetTick() - rx_mode_until) >= 0)
		led_set_mode(LED_MODE_STANDBY);
	if (led_mode == LED_MODE_STANDBY || led_mode == LED_MODE_RX) {
		uint32_t now = HAL_GetTick();
		if (now - led_last_toggle >= led_period_ms) {
			LED_TOGGLE();
			led_last_toggle = now;
		}
	}
}

/* Flash ops */
static void erase_flash(void) {
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef E = { .TypeErase = FLASH_TYPEERASE_PAGES,
			.PageAddress = APP_BASE_ADDR, .NbPages = MAX_APP_PAGES };
	uint32_t perr;
	(void) HAL_FLASHEx_Erase(&E, &perr);
	HAL_FLASH_Lock();
}
static void flash_consume_bytes(const uint8_t *data, uint32_t len) {
	HAL_FLASH_Unlock();
	for (uint32_t i = 0; i < len; i++) {
		wbuf[wlen++] = data[i];
		if (wlen == 4) {
			uint32_t word = *(uint32_t*) wbuf;
			(void) HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress,
					word);
			flashAddress += 4;
			wlen = 0;
		}
	}
	HAL_FLASH_Lock();
}
static void flash_flush_tail(void) {
	if (wlen == 0)
		return;
	while (wlen < 4)
		wbuf[wlen++] = 0x00;
	HAL_FLASH_Unlock();
	uint32_t word = *(uint32_t*) wbuf;
	(void) HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress, word);
	flashAddress += 4;
	HAL_FLASH_Lock();
	wlen = 0;
}
static int is_app_valid(void) {
	uint32_t sp = *(uint32_t*) APP_BASE_ADDR, rh = *(uint32_t*) (APP_BASE_ADDR
			+ 4);
	return ((sp & 0x2FFE0000U) == 0x20000000U)
			&& (rh != 0xFFFFFFFFU && rh != 0x00000000U);
}

/* === CAN TX: blocking + retry (กัน ACK หลุด) === */
static HAL_StatusTypeDef can_tx_blocking(uint32_t std_id, const uint8_t *data,
		uint8_t dlc, uint32_t timeout_ms) {
	CAN_TxHeaderTypeDef tx;
	uint32_t mb;
	tx.StdId = std_id;
	tx.IDE = CAN_ID_STD;
	tx.RTR = CAN_RTR_DATA;
	tx.DLC = dlc;
	uint32_t t0 = HAL_GetTick();
	while ((HAL_GetTick() - t0) < timeout_ms) {
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
			HAL_StatusTypeDef st = HAL_CAN_AddTxMessage(&hcan, &tx,
					(uint8_t*) data, &mb);
			if (st == HAL_OK)
				return HAL_OK;
		}
		__NOP();
	}
	return HAL_TIMEOUT;
}
static void send_ack(void) {
	uint8_t d = 0xAA;
	if (can_tx_blocking(CAN_ID_ACK, &d, 1, 3) != HAL_OK)
		(void) can_tx_blocking(CAN_ID_ACK, &d, 1, 3);
	// debug_printf("ACK @idx=%lu bytes=%lu\r\n",(unsigned long)expected_frame_index,(unsigned long)bytes_written);
}
static void send_ready(void) {
	uint8_t d = 0xAC;
	(void) can_tx_blocking(CAN_ID_ACK, &d, 1, 3);
}
static void send_nack(uint16_t redo_index) {
	uint8_t d[2] =
			{ (uint8_t) (redo_index & 0xFF), (uint8_t) (redo_index >> 8) };
	(void) can_tx_blocking(CAN_ID_NACK, d, 2, 3);
	debug_printf("NACK -> redo idx=%u\r\n", (unsigned) redo_index);
}

/* ISR: push-only */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h) {
	CAN_RxHeaderTypeDef hdr;
	uint8_t data[8];
	HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &hdr, data);
	rb_push(&hdr, data);
}

/* Jump helpers */
static void deinitEverything(void) {
	HAL_CAN_DeInit(&hcan);
	HAL_UART_DeInit(&huart1);
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOD_CLK_DISABLE();
	HAL_RCC_DeInit();
	HAL_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
}
static void jump_to_app(void) {
	__disable_irq();
	for (uint8_t i = 0; i < 8; i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}
	deinitEverything();
	SCB->VTOR = APP_BASE_ADDR;
	uint32_t sp = *(uint32_t*) APP_BASE_ADDR, entry =
			*(uint32_t*) (APP_BASE_ADDR + 4);
	LED_OFF();
	asm volatile("msr msp, %[sp]\n\tcpsie i\n\tbx %[entry]\n\t"::[sp]"r"(sp),[entry]"r"(entry):);
}

/* ============================== MAIN ============================== */
int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_CAN_Init();
	MX_USART1_UART_Init();
	led_init();
	led_set_mode(LED_MODE_STANDBY);

	CAN_ConfigFilters();
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	debug_printf("Bootloader (CAN) started. BL=%uKB, APP@0x%08lX\r\n",
			BOOTLOADER_SIZE_KB, (unsigned long) APP_BASE_ADDR);
	debug_printf("BL BATCH=%u\r\n", (unsigned) BATCH_SIZE);

	/* รอ START; ถ้าไม่มาและแอป valid ให้ jump */
	uint32_t deadline = HAL_GetTick() + 5000;
	can_frame_t f;
	while (HAL_GetTick() < deadline && !firmwareUpdating) {
		while (rb_pop(&f)) {
			if (f.hdr.StdId == CAN_ID_START) {
				if (f.hdr.DLC != 8 || f.data[0] != 'S' || f.data[1] != 'T'
						|| f.data[2] != 'A' || f.data[3] != 'R') {
					debug_printf("START invalid payload\r\n");
					continue;
				}

				/* reset state + เคลียร์คิวกันเฟรมค้าง */
				rx_tail = rx_head;
				expected_frame_index = 0;
				frame_count = 0;
				waiting_resend = 0;
				wlen = 0;

				expected_size = *(uint32_t*) &f.data[4];
				current_crc = 0xFFFFFFFF;
				bytes_written = 0;
				flashAddress = APP_BASE_ADDR;
				firmwareUpdating = 1;

				/* START handshake: ACK → erase → READY */
				send_ack();
				led_set_mode(LED_MODE_BUSY);
				erase_flash();
				send_ready();

				debug_printf("START ok size=%lu\r\n", expected_size);
			}
		}
		led_task();
	}

	if (!firmwareUpdating && is_app_valid()) {
		debug_printf("Valid app found. Jumping...\r\n");
		HAL_Delay(30);
		jump_to_app();
	} else if (!firmwareUpdating) {
		debug_printf("No valid app or start update.\r\n");
	}

	while (1) {
		while (rb_pop(&f)) {

			/* -------- DATA -------- */
			if (f.hdr.StdId == CAN_ID_DATA && firmwareUpdating) {
				led_set_mode(LED_MODE_RX);

				if (f.hdr.DLC < 2) {
					debug_printf("DLC<2 @DATA\r\n");
					continue;
				}
				uint16_t frame_idx = (uint16_t) (f.data[0] | (f.data[1] << 8));

				if (waiting_resend) {
					if (frame_idx != expected_frame_index)
						continue;   // still waiting
					waiting_resend = 0;
					frame_count = 0;               // got the one we asked
				} else {
					if (frame_idx != expected_frame_index) {
						debug_printf("IDX MISMATCH got=%u exp=%u DLC=%u\r\n",
								frame_idx, expected_frame_index, f.hdr.DLC);
						send_nack((uint16_t) expected_frame_index);
						waiting_resend = 1;
						continue;
					}
				}

				expected_frame_index++;

				/* payload ≤6 */
				uint8_t *payload = &f.data[2];
				uint8_t payload_len = (uint8_t) (f.hdr.DLC - 2);
				if (payload_len > 6)
					payload_len = 6;

				uint32_t remain =
						(expected_size > bytes_written) ?
								(expected_size - bytes_written) : 0;
				uint8_t use = (uint8_t) MIN((uint32_t )payload_len, remain);
				if (use > 0) {
					flash_consume_bytes(payload, use);
					current_crc = crc32_update(current_crc, payload, use);
					bytes_written += use;
					if (expected_size) {
						uint32_t p = (bytes_written * 100U) / expected_size;
						debug_printf("Progress: %lu%%\r\n", p);
					}
				}

				/* ACK policy */
#if DEBUG_ACK_EVERY_FRAME
        send_ack();
#else
				if (!waiting_resend) {
					frame_count++;
					if (frame_count >= BATCH_SIZE
							|| bytes_written >= expected_size) {
						send_ack();
						frame_count = 0;
					}
				}
#endif
			}

			/* -------- END -------- */
			else if (f.hdr.StdId == CAN_ID_END && firmwareUpdating) {
				if (f.hdr.DLC != 8 || f.data[0] != 'E' || f.data[1] != 'N'
						|| f.data[2] != 'D' || f.data[3] != 0) {
					debug_printf("END invalid payload\r\n");
					continue;
				}
				expected_crc = *(uint32_t*) &f.data[4];

				flash_flush_tail();
				firmwareUpdating = 0;

				/* dump head/tail debug */
				debug_printf("HEAD:\r\n");
				for (uint32_t i = 0; i < 32 && i < expected_size; i++) {
					uint8_t b = *(__IO uint8_t*) (APP_BASE_ADDR + i);
					debug_printf("%02X", b);
				}
				debug_printf("\r\nTAIL:\r\n");
				uint32_t tail_start =
						(expected_size > 32) ? (expected_size - 32) : 0;
				for (uint32_t i = 0; i < 32 && (tail_start + i) < expected_size;
						i++) {
					uint8_t b =
							*(__IO uint8_t*) (APP_BASE_ADDR + tail_start + i);
					debug_printf("%02X", b);
				}
				debug_printf("\r\n");

				debug_printf(
						"END exp_crc=0x%08lX calc_stream=0x%08lX size=%lu\r\n",
						expected_crc, current_crc, bytes_written);

				/* CRC from flash */
				uint8_t buf32[32];
				uint32_t crc_flash = 0xFFFFFFFF, addr = APP_BASE_ADDR;
				for (uint32_t rem = expected_size; rem > 0;) {
					uint32_t n = MIN(rem, (uint32_t )sizeof(buf32));
					memcpy(buf32, (const void*) addr, n);
					crc_flash = crc32_update(crc_flash, buf32, n);
					addr += n;
					rem -= n;
				}
				debug_printf("CRC from flash: 0x%08lX\r\n", crc_flash);

				if (bytes_written == expected_size && is_app_valid()
						&& crc_flash == expected_crc) {
					debug_printf("OK -> jump\r\n");
					HAL_Delay(30);
					jump_to_app();
				} else {
					debug_printf("FAIL: size/crc/app invalid\r\n");
					LED_OFF();
					led_set_mode(LED_MODE_STANDBY);
				}
			}
		}
		led_task();
	}
}

/* ===== HAL init ===== */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		Error_Handler();

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
		Error_Handler();
}
static void MX_CAN_Init(void) {
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 2;               // 1Mbps @ APB1=36MHz
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = ENABLE; // สำคัญ
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
		Error_Handler();
}
static void CAN_ConfigFilters(void) {
	CAN_FilterTypeDef f = { 0 };
	f.FilterMode = CAN_FILTERMODE_IDMASK;
	f.FilterScale = CAN_FILTERSCALE_32BIT;
	f.FilterFIFOAssignment = CAN_RX_FIFO0;
	f.FilterActivation = ENABLE;

	f.FilterBank = 0;
	f.FilterIdHigh = (uint16_t) (0x7FFu << 5);
	f.FilterIdLow = 0;
	f.FilterMaskIdHigh = (uint16_t) (0x7FFu << 5);
	f.FilterMaskIdLow = 0;
	HAL_CAN_ConfigFilter(&hcan, &f);
	f.FilterBank = 1;
	f.FilterIdHigh = (uint16_t) (0x100u << 5);
	HAL_CAN_ConfigFilter(&hcan, &f);
	f.FilterBank = 2;
	f.FilterIdHigh = (uint16_t) (0x7FEu << 5);
	HAL_CAN_ConfigFilter(&hcan, &f);
}
static void MX_USART1_UART_Init(void) {
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = 16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
		Error_Handler();
}
static void MX_GPIO_Init(void) {
	__HAL_RCC_GPIOD_CLK_ENABLE();__HAL_RCC_GPIOA_CLK_ENABLE(); /* PC13 init ใน led_init() */
}

void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){ (void)file; (void)line; }
#endif
