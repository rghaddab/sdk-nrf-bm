/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdint.h>

#include <ble_gap.h>
#include <nrf_soc.h>
#include <bm/bm_timer.h>
#include <bm/fs/bm_zms.h>
#include <bm/softdevice_handler/nrf_sdh.h>
#include <bm/softdevice_handler/nrf_sdh_ble.h>
#include <bm/storage/bm_storage_backends.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include <board-config.h>

LOG_MODULE_REGISTER(sample, CONFIG_ZMS_TEST_SAMPLE_LOG_LEVEL);

#define PM_STORAGE_NODE DT_NODELABEL(peer_manager_partition)
#define PM_SECTOR_SIZE 1024

#define DO_STUFF_INTERVAL_MS 2000

static struct bm_timer do_stuff_timer;
static struct bm_zms_fs fs;

static const uint32_t entry_id_a = 43;
static uint8_t pattern_a = 0x11;
static uint8_t data_a[118] __attribute((aligned(4)));

static const uint32_t entry_id_b = 42;
static uint8_t pattern_b = 0xAA;
static uint8_t data_b[20] __attribute((aligned(4)));

static volatile bool write_notif;
static volatile bool mount_notif;

static void wait_for_write(void)
{
        while (!write_notif) {
		log_flush();
                k_cpu_idle();
        }
        write_notif = false;
}

static void wait_for_mount(void)
{
	while (!mount_notif) {
		log_flush();
		k_cpu_idle();
	}
	mount_notif = false;
}

static void do_stuff_timeout_handler(void *context)
{
	long ret;
	uint8_t read_buf[128];

	ARG_UNUSED(context);

	memset(read_buf, 0xFF, sizeof(read_buf));

	/* Swap pattern for entry a. */
	pattern_a = (pattern_a == 0x22) ? 0x33 : 0x22;
	memset(&data_a, pattern_a, sizeof(data_a));

	/* Write entry a with updated pattern. */
	ret = bm_zms_write(&fs, entry_id_a, &data_a[0], sizeof(data_a));
	if (ret < 0) {
		LOG_ERR("Failed to write entry a, err %ld", ret);
	}

	/* Read entry b. */
	ret = bm_zms_read(&fs, entry_id_b, &read_buf[0], sizeof(read_buf));
	if (ret < 0) {
		LOG_ERR("Failed to read entry b, err %ld", ret);
	} else {
		const int len = (ret > sizeof(read_buf)) ? sizeof(read_buf) : ret;

		if (memcmp(&read_buf[0], &data_b[0], len) != 0) {
			LOG_ERR("Unexpected read-write difference for entry b");
		}
	}

	/* Swap pattern for entry b. */
	pattern_b = (pattern_b == 0xAA) ? 0xBB : 0xAA;
	memset(&data_b, pattern_b, sizeof(data_b));

	/* Write entry b with updated pattern. */
	ret = bm_zms_write(&fs, entry_id_b, &data_b[0], sizeof(data_b));
	if (ret < 0) {
		LOG_ERR("Failed to write entry b, err %ld", ret);
	}
}

static void bm_zms_evt_handler(const struct bm_zms_evt *evt)
{
	switch (evt->evt_type) {
	case BM_ZMS_EVT_MOUNT:
		LOG_INF("ZMS mounted, result %d", evt->result);
		mount_notif = true;
		if (evt->result) {
			LOG_ERR("Failed bm_zms_mount, err %d", evt->result);
		}
		break;
	
	case BM_ZMS_EVT_WRITE:
		LOG_INF("Write event, id %u, result %d", evt->id, evt->result);
		write_notif = true;
		if (evt->result) {
			LOG_ERR("Failed bm_zms_write, err %d", evt->result);
		}
		break;
	
	case BM_ZMS_EVT_DELETE:
		if (evt->result) {
			LOG_ERR("Failed bm_zms_delete err %d", evt->result);
		}
		break;
	
	default:
		LOG_WRN("Unhandled bm_zms event, type %d, result %d, id %d",
			evt->evt_type, evt->result, evt->id);
		break;
	}
}

int main(void)
{
	int err;
	struct bm_zms_fs_config zms_config = {
		.offset = DT_REG_ADDR(PM_STORAGE_NODE),
		.sector_size = PM_SECTOR_SIZE,
		.sector_count = (DT_REG_SIZE(PM_STORAGE_NODE) / PM_SECTOR_SIZE),
		.evt_handler = bm_zms_evt_handler,
		.storage_api = &bm_storage_sd_api,
	};

	LOG_INF("ZMS testing sample");

	err = nrf_sdh_enable_request();
	if (err) {
		LOG_ERR("Failed to enable SoftDevice, err %d", err);
		goto idle;
	}

	err = nrf_sdh_ble_enable(CONFIG_NRF_SDH_BLE_CONN_TAG);
	if (err) {
		LOG_ERR("Failed to enable Bluetooth LE, err %d", err);
		goto idle;
	}

	LOG_INF("SoftDevice enabled");

	err = bm_zms_mount(&fs, &zms_config);
	if (err) {
		LOG_ERR("Failed to initialize NVM storage, err %d", err);
		goto idle;
	}

	/* Wait for zms initialization. */
	wait_for_mount();

	err = bm_timer_init(&do_stuff_timer, BM_TIMER_MODE_REPEATED, do_stuff_timeout_handler);
	if (err) {
		LOG_ERR("Failed to initialize do_stuff timer, err %d", err);
		goto idle;
	}

	err = bm_timer_start(&do_stuff_timer, BM_TIMER_MS_TO_TICKS(DO_STUFF_INTERVAL_MS), NULL);
	if (err) {
		LOG_ERR("Failed to start do_stuff timer, err %d", err);
		goto idle;
	}
idle:
	while (true) {
		log_flush();

		k_cpu_idle();
	}

	return 0;
}
