/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <stdbool.h>
#include <tegra-can.h>
#include <tegra-mttcan.h>
#include <can-tegra-hw.h>
#include <tke-tegra.h>

#include "can-app.h"

/*
 * This app uses CAN0 as transmitter and CAN1 as receiver, both the controllers
 * communicate using CAN transceivers. CAN driver also creates transmit element
 * on every transmit or bus state change, this app also uses these events
 * accordingly to follow the CAN Bus state and transmit status.
 * Follow can-app.txt for more information
 */

#define CAN0_TEST_CONTROLLER		&tegra_gttcan[0]
#define CAN1_TEST_CONTROLLER		&tegra_gttcan[1]
#define CAN_TEST_DELAY			10000

static void can_rx_test_print(struct ttcan_controller *ttcan,
			      struct mttcanfd_frame *cf)
{
	int msgid, dlen, i;
	struct canfd_frame frame;

	/* Process debug print only if packet is a CAN RX message */
	if (cf->cmdid != MTTCAN_MSG_RX) {
		printf("Message received with command id (0x%02x) different"
			" than MTTCAN_MSG_RX at CAN %lu\r\n", cf->cmdid,
			ttcan->id);
		return;
	}

	/* Update canfd_frame based on data from CAN message RAM */
	ttcan_read_rx_msg_ram(ttcan, (uint32_t)&cf->payload.data[0], &frame);
	msgid = frame.can_id;
	dlen = frame.d_len;
	printf("Message received at CAN %lu\r\n", ttcan->id);
	printf("Message ID: 0x%02x, Message data length: %d\r\n", msgid, dlen);
	printf("Message Data:");
	for (i = 0; i < frame.d_len; i++) {
		if (!(i & 7))
			printf("\r\n");
		printf("0x%02x ", frame.data[i]);
	}
	printf("\r\n");
}

/* Waits on CAN RX queue for messages */
static portTASK_FUNCTION(can1_rx_test, pvParameters)
{
	struct ttcan_controller *ttcan = (struct ttcan_controller *)pvParameters;
	struct mttcanfd_frame cf;

	while (1) {
		/* Block on CAN ISR to get data */
		if (!tegra_can_receive(ttcan, &cf, portMAX_DELAY))
			continue;
		can_rx_test_print(ttcan, &cf);
	}
}

static void can_process_txevent(uint32_t elm0, uint32_t elm1)
{
	uint8_t id_size = (elm0 >> 30) & 0x1;
	uint32_t id = id_size ? elm0 & 0x1fffffff : (elm0 >> 18) & 0x7ff;
	uint8_t event_type = (elm1 >> 22) & 0x3;
	uint8_t can_frame_format = (elm1 >> 21) & 0x1;
	uint8_t can_brs = (elm1 >> 20) & 0x1;

	printf("Transmit event element information:\r\n");
	printf("Message ID: 0x%x, Event Type: %s, CAN Frame: %s, %s, %s\r\n",
		(unsigned int)id, (event_type == 1) ? "Tx event" :
		(event_type == 2) ? "Tx in spite of cancellation" : "Reserved",
		(can_frame_format == 1) ? "CAN FD" : "Standard Frame",
		(id_size == 1) ? "29bit ID" : "11bit ID",
		(can_brs == 1) ? "BRS" : "No BRS");
}

/*
 * CAN0 creates transmit event elements on each mttcan_transfer, this task reads
 * out the event elements from the internal CAN message RAM.
 */
static portTASK_FUNCTION(can0_tx_event, pvParameters)
{
	struct ttcan_controller *ttcan = (struct ttcan_controller *)pvParameters;
	struct mttcanfd_frame cf;

	while (1) {
		/* Block on CAN ISR to get data */
		if (!tegra_can_receive(ttcan, &cf, portMAX_DELAY))
			continue;

		switch (cf.cmdid) {
		case MTTCAN_MSG_TX_COMPL:
			printf("Transmission complete event\r\n");
			break;
		case MTTCAN_MSG_TXEVT:
			/*
			 * Each transmit event element is two words long, we
			 * will selectively parse fields of both the elements
			 */
			can_process_txevent(cf.payload.data[0],
					    cf.payload.data[1]);
			break;
		case MTTCAN_MSG_BERR_CHG:
			printf("Bus Error\r\n");
			break;
		case MTTCAN_MSG_STAT_CHG:
			printf("Bus state change\r\n");
			break;
		default:
			printf("Commnad id: 0x%x not processed\r\n", cf.cmdid);
			break;
		}
	}
}

static void can_test_transmit(struct mttcanfd_frame *fd,
			      struct ttcan_controller *ttcan, bool is_xtd,
			      bool is_can_fd, bool is_brs)
{
	if (is_xtd)
		fd->payload.frame.can_id |= (CAN_FMT);
	else
		fd->payload.frame.can_id &= ~(CAN_FMT);

	if (is_can_fd && is_brs)
		fd->payload.frame.flags |= (CAN_FD_FLAG) | (CAN_BRS_FLAG);
	else if (is_can_fd)
		fd->payload.frame.flags |= (CAN_FD_FLAG);
	else
		fd->payload.frame.flags &= ~((CAN_FD_FLAG) | (CAN_BRS_FLAG));

	fd->payload.frame.tstamp = tegra_tke_get_usec();
	mttcan_transfer(ttcan, fd);
	printf("Transmited message from CAN %lu\r\n", ttcan->id);
	vTaskDelay(CAN_TEST_DELAY);
}

static portTASK_FUNCTION(can0_xmit_test, pvParameters)
{
	struct ttcan_controller *ttcan = (struct ttcan_controller *)pvParameters;
	struct mttcanfd_frame fd;

	memset(&fd, 0, sizeof(fd));
	/* cmdid information can be found in tegra-can.h */
	fd.cmdid = MTTCAN_MSG_TX;
	/*
	 * can_id and data are just random numbers, information regarding can_id
	 * and flags bit fields can be found in tegra-can.h. Please note that
	 * BRS mode is not supported for now.
	 */
	fd.payload.frame.can_id = 0xa5;
	fd.payload.frame.d_len = 2;
	fd.payload.frame.flags = 0x0;
	fd.payload.frame.data[0] = 0xaa;
	fd.payload.frame.data[1] = 0x55;

	while (1) {
		/* Normal CAN frame with 11 bit identifier */
		can_test_transmit(&fd, ttcan, false, false, false);
		/* Normal CAN frame with 29 bit identifier */
		can_test_transmit(&fd, ttcan, true, false, false);
		/* CAN FD frame with 11 bit identifier */
		can_test_transmit(&fd, ttcan, false, true, false);
		/* CAN FD frame with 29 bit identifier */
		can_test_transmit(&fd, ttcan, true, true, false);
		/* CAN FD BRS frame with 11 bit identifier */
		can_test_transmit(&fd, ttcan, false, true, true);
		/* CAN FD BRS frame with 29 bit identifier */
		can_test_transmit(&fd, ttcan, true, true, true);
	}
}

static void can_deinit(struct ttcan_controller *ttcan)
{
	int result;
	/* Put CAN controller in reset and disable interrupts */
	result = mttcan_controller_enable(ttcan, false);
	if (result)
		error_hook("Unable to deinit CAN controllers\r\n");

	/* Free internal queue and semaphore */
	mttcan_controller_deinit(ttcan);
}

static BaseType_t can_init(struct ttcan_controller *ttcan)
{
	BaseType_t ret;
	int result;

	ret = mttcan_controller_init(ttcan);
	if (ret != pdPASS)
		return ret;

	result = mttcan_controller_enable(ttcan, true);
	if (!result)
		return pdPASS;
	else
		return pdFAIL;
}

void can_app_init(void)
{
	BaseType_t ret;
	TaskHandle_t can0_xmit_handle, can0_tx_eve_handle, can1_rx_handle;

	/* Setup CAN0 and CAN1 controllers */
	ret = can_init(CAN0_TEST_CONTROLLER);
	if (ret != pdPASS) {
		error_hook("Unable to initialize CAN0 controllers\r\n");
		return;
	}

	ret = can_init(CAN1_TEST_CONTROLLER);
	if (ret != pdPASS) {
		error_hook("Unable to initialize CAN1 controllers\r\n");
		goto can0_deinit_err;
	}

	/* CAN0 TX */
	ret = xTaskCreate(can0_xmit_test, 0, 512, CAN0_TEST_CONTROLLER,
			  tskIDLE_PRIORITY, &can0_xmit_handle);
	if (ret != pdPASS) {
		error_hook("CAN0: xTaskCreate for transmit failed\r\n");
		goto can_deinit_err;
	}

	ret = xTaskCreate(can0_tx_event, 0, 512, CAN0_TEST_CONTROLLER,
			  tskIDLE_PRIORITY, &can0_tx_eve_handle);
	if (ret != pdPASS) {
		error_hook("CAN0: xTaskCreate for receive failed\r\n");
		goto xmit_test_delete;
	}

	/* CAN1 RX */
	ret = xTaskCreate(can1_rx_test, 0, 512, CAN1_TEST_CONTROLLER,
			  tskIDLE_PRIORITY, &can1_rx_handle);
	if (ret != pdPASS) {
		error_hook("CAN1: xTaskCreate for receive failed\r\n");
		goto can_task_delete;
	}

	return;

can_task_delete:
	vTaskDelete(can0_tx_eve_handle);
xmit_test_delete:
	vTaskDelete(can0_xmit_handle);
can_deinit_err:
	can_deinit(CAN1_TEST_CONTROLLER);
can0_deinit_err:
	can_deinit(CAN0_TEST_CONTROLLER);
}
