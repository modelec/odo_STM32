/*
 * commSTM.c
 *
 *  Created on: May 13, 2025
 *      Author: maxch
 */




#include "commSTM.h"
#include "usbd_cdc.h"
#include <string.h>
#include <stdio.h>

// ⚠️ fourni par CubeMX dans usb_device.c
extern USBD_HandleTypeDef hUsbDeviceFS;

// Buffer de réception circulaire
#define RX_BUFFER_SIZE 256
static char usb_rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t usb_rx_index = 0;
static volatile uint8_t data_ready = 0;

static char parse_buffer[RX_BUFFER_SIZE]; // Pour parser la ligne complète

// Fonction à appeler depuis CDC_Receive_FS
void USB_Comm_OnReceive(uint8_t* Buf, uint32_t Len) {
    if (Len + usb_rx_index >= RX_BUFFER_SIZE) {
        // Trop de données, on reset le buffer
        usb_rx_index = 0;
        return;
    }

    for (uint32_t i = 0; i < Len; i++) {
        char c = Buf[i];
        if (c == '\n' || c == '\r') {
            // Fin de commande
            usb_rx_buffer[usb_rx_index] = '\0';
            memcpy(parse_buffer, usb_rx_buffer, usb_rx_index + 1);
            usb_rx_index = 0;
            data_ready = 1;
            return;
        } else {
            usb_rx_buffer[usb_rx_index++] = c;
        }
    }
}

// Répondre via USB
static void USB_Comm_Send(const char* message) {
    if (message != NULL) {
        CDC_Transmit_FS((uint8_t*)message, strlen(message));
    }
}

// Analyse et réponse aux commandes reçues
void USB_Comm_Process(void) {
    if (!data_ready) return;

    data_ready = 0;

    if (strncmp(parse_buffer, "GET;POS", 7) == 0) {
        USB_Comm_Send("SET;POS;123;456;1.57\n");
    }
    else if (strncmp(parse_buffer, "GET;SPEED", 9) == 0) {
        USB_Comm_Send("SET;SPEED;12;34;0.1\n");
    }
    else if (strncmp(parse_buffer, "GET;PID", 7) == 0) {
        USB_Comm_Send("SET;PID;1.0;0.1;0.05\n");
    }
    else if (strncmp(parse_buffer, "SET;POS;", 8) == 0) {
        // Tu peux parser les valeurs et mettre à jour ta position ici
        USB_Comm_Send("OK;POS\n");
    }
    else {
        USB_Comm_Send("KO;UNKNOWN\n");
    }
}
