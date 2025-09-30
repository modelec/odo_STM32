/*
 * commSTM.c
 *
 *  Created on: May 13, 2025
 *      Author: maxch
 */

#include "CommCallbacks.h"
#include "commSTM.h"

#include "modelec.h"
#include "usbd_cdc_if.h"

// fourni par CubeMX dans usb_device.c
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
        usb_rx_index = 0;
        return;
    }

    for (uint32_t i = 0; i < Len; i++) {
        char c = Buf[i];
        if (c == '\n' || c == '\r') {
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

    char *token = strtok(parse_buffer, ";");
    if (!token) return;

    if (strcmp(token, "GET") == 0) {
        token = strtok(NULL, ";");
        if (!token) return;

        if (strcmp(token, "POS") == 0) {
            float x, y, t;
            Comm_GetPos(x, y, t);
            char response[64];
            snprintf(response, sizeof(response), "SET;POS;%.4f;%.4f;%.4f\n", x, y, t);
            USB_Comm_Send(response);
        }
        else if (strcmp(token, "SPEED") == 0) {
            float vx, vy, omega;
            Comm_GetSpeed(vx, vy, omega);
            char response[64];
            snprintf(response, sizeof(response), "SET;SPEED;%.4f;%.4f;%.4f\n", vx, vy, omega);
            USB_Comm_Send(response);
        }
        else if (strcmp(token, "PID") == 0) {
            char* pid = strtok(NULL, ";");
            float p, i, d;
            if (Comm_GetPID(pid, p, i, d)) {
                char response[64];
                snprintf(response, sizeof(response), "SET;PID;%.4f;%.4f;%.4f\n", p, i, d);
                USB_Comm_Send(response);
            }
            else {
                USB_Comm_Send("KO;PID;UNKNOWN\n");
            }
        }
        else if (strcmp(token, "DIST") == 0) {
            token = strtok(NULL, ";");
            if (!token) return;
            int n = atoi(token);
            float dist = Comm_GetDistance(n);
            char response[64];
            snprintf(response, sizeof(response), "SET;DIST;%d;%.2f\n", n, dist);
            USB_Comm_Send(response);
        }
        else {
            USB_Comm_Send("KO;UNKNOWN\n");
        }
    }
    else if (strcmp(token, "SET") == 0) {
        token = strtok(NULL, ";");
        if (!token) return;

        if (strcmp(token, "POS") == 0) {
            float x = atof(strtok(NULL, ";"));
            float y = atof(strtok(NULL, ";"));
            float t = atof(strtok(NULL, ";"));
            Comm_SetPos(x, y, t);
            USB_Comm_Send("OK;POS\n");
        }
        else if (strcmp(token, "PID") == 0) {
            char* pid = strtok(NULL, ";");
            float p = atof(strtok(NULL, ";"));
            float i = atof(strtok(NULL, ";"));
            float d = atof(strtok(NULL, ";"));

            if (Comm_SetPID(pid, p, i, d)) {
                USB_Comm_Send("OK;PID\n");
            }
            else {
                USB_Comm_Send("KO;PID;UNKNOWN\n");
            }
        }
        else if (strcmp(token, "WAYPOINT") == 0) {

            while (true) {

                char* idTok = strtok(nullptr, ";");
                char* typeTok = strtok(nullptr, ";");
                char* xTok = strtok(nullptr, ";");
                char* yTok = strtok(nullptr, ";");
                char* tTok = strtok(nullptr, ";");

                if (!idTok || !typeTok || !xTok || !yTok || !tTok) {
                    break;
                }

                int id = atoi(idTok);
                int type = atoi(typeTok);
                float x = atof(xTok);
                float y = atof(yTok);
                float theta = atof(tTok);

                Comm_AddWaypoint(id, type, x, y, theta);
            }
            USB_Comm_Send("OK;WAYPOINT\n");
        }
        else if (strcmp(token, "START") == 0) {
        	int val = atoi(strtok(NULL, ";"));
        	Comm_StartOdometry(val != 0);

        	USB_Comm_Send("OK;START\n");
        }
        else if (strcmp(token, "MOTOR") == 0) {
            float left = atof(strtok(NULL, ";"));
            float right = atof(strtok(NULL, ";"));

            if (left < -PWM_MAX || left > PWM_MAX || right < -PWM_MAX || right > PWM_MAX) {
                USB_Comm_Send("KO;MOTOR;RANGE\n");
                return;
            }

            Comm_SetPWM(left, right);

            USB_Comm_Send("OK;MOTOR\n");
        }
        else {
            USB_Comm_Send("KO;UNKNOWN\n");
        }
    }
    else {
        USB_Comm_Send("KO;UNKNOWN\n");
    }
}
