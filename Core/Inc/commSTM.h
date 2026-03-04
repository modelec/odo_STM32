/*
 * commSTM.h
 *
 *  Created on: May 13, 2025
 *      Author: Modelec
 */

#ifndef INC_COMMSTM_H_
#define INC_COMMSTM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// À appeler régulièrement pour parser ce qui a été reçu
void USB_Comm_Process(void);

// À appeler dans CDC_Receive_FS (depuis usbd_cdc_if.c)
void USB_Comm_OnReceive(uint8_t* Buf, uint32_t Len);

#ifdef __cplusplus
}
#endif

#endif /* INC_COMMSTM_H_ */
