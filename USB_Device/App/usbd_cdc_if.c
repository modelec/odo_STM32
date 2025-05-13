/* usbd_cdc_if.c */

#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
}

static int8_t CDC_DeInit_FS(void)
{
  return (USBD_OK);
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  return (USBD_OK);
}

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  Buf[*Len] = '\0';
  USB_CDC_ProcessCommand((char*)Buf);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  return (USBD_OK);
}

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  if (hUsbDeviceFS.pClassData == NULL) return USBD_FAIL;
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

/* Command parser */
void USB_CDC_ProcessCommand(const char* command)
{
  char response[APP_TX_DATA_SIZE];

  if (strncmp(command, "GET;POS", 7) == 0) {
    snprintf(response, sizeof(response), "SET;POS;100;200;1.57\r\n");
  } else if (strncmp(command, "GET;SPEED", 9) == 0) {
    snprintf(response, sizeof(response), "SET;SPEED;10;20;0.5\r\n");
  } else if (strncmp(command, "GET;DIST;", 9) == 0) {
    int tof_id = atoi(command + 9);
    snprintf(response, sizeof(response), "SET;DIST;%d;123\r\n", tof_id);
  } else if (strncmp(command, "GET;PID", 7) == 0) {
    snprintf(response, sizeof(response), "SET;PID;1.0;0.5;0.1\r\n");
  } else if (strncmp(command, "SET;POS;", 9) == 0 ||
             strncmp(command, "SET;PID;", 9) == 0 ||
             strncmp(command, "SET;START;", 11) == 0 ||
             strncmp(command, "SET;WAYPOINT;", 13) == 0) {
    snprintf(response, sizeof(response), "OK;%s\r\n", command + 4);
  } else {
    snprintf(response, sizeof(response), "KO;%s\r\n", command);
  }

  CDC_Transmit_FS((uint8_t*)response, strlen(response));
}
