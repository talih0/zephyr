/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

void blink(uint32_t sleep_ms, uint32_t id)
{
	int cnt = 0;

	while (1) {
		volatile uint8_t *data = malloc(100);
		if (data != NULL) {
			data[0] = 1;
			free((void*)data);
		}

		k_msleep(sleep_ms);
		cnt++;
		printf("Count %d\n", cnt);
	}
}

void blink0(void)
{
	blink(200, 0);
}

void blink1(void)
{
	blink(100, 1);
}

K_THREAD_DEFINE(blink0_id, STACKSIZE, blink0, NULL, NULL, NULL,
		PRIORITY, 0, 0);
K_THREAD_DEFINE(blink1_id, STACKSIZE, blink1, NULL, NULL, NULL,
		PRIORITY + 1, 0, 0);
