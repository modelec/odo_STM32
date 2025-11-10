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

#include <cmath>

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
            if (usb_rx_index == 0) continue; // ignore empty lines

            usb_rx_buffer[usb_rx_index] = '\0';
            memcpy(parse_buffer, usb_rx_buffer, usb_rx_index + 1);
            usb_rx_index = 0;
            data_ready = 1;

            USB_Comm_Process();
        } else {
            if (usb_rx_index < RX_BUFFER_SIZE - 1) {
                usb_rx_buffer[usb_rx_index++] = c;
            } else {
                usb_rx_index = 0;
            }
        }
    }}

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
            float p, i, d, v_min, v_max;
            if (Comm_GetPID(pid, p, i, d, v_min, v_max)) {
                char response[64];
                snprintf(response, sizeof(response), "SET;PID;%.4f;%.4f;%.4f;%.4f,%.4f\n", p, i, d, v_min, v_max);
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
        else if (strcmp(token, "FREQUENCY")) {
            token = strtok(NULL, ";");
            if (!token) return;
            uint32_t freq;
            Comm_GetPublishFrequency(freq);
            char response[64];
            snprintf(response, sizeof(response), "SET;FREQUENCY;%ld\n", freq);
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
            if (!pid) {
                USB_Comm_Send("KO;PID;MISSING_NAME\n");
                return;
            }

            char* p_str = strtok(NULL, ";");
            char* i_str = strtok(NULL, ";");
            char* d_str = strtok(NULL, ";");

            if (!p_str || !i_str || !d_str) {
                char msg[64];
                snprintf(msg, sizeof(msg), "KO;PID;%s;MISSING_VALUES\n", pid);
                USB_Comm_Send(msg);
                return;
            }

            float p = atof(p_str);
            float i = atof(i_str);
            float d = atof(d_str);

            // Optional parameters
            char* out_min_str = strtok(NULL, ";");
            char* out_max_str = strtok(NULL, ";");

            bool has_out_min = (out_min_str != nullptr);
            bool has_out_max = (out_max_str != nullptr);

            float out_min = has_out_min ? atof(out_min_str) : 0.0f;
            float out_max = has_out_max ? atof(out_max_str) : 0.0f;

            bool success = false;

            // Call with or without output limits
            if (has_out_min && has_out_max) {
                success = Comm_SetPID(pid, p, i, d, out_min, out_max);
            } else {
                success = Comm_SetPID(pid, p, i, d, NAN, NAN);
            }

            char msg[64];
            if (success) {
                snprintf(msg, sizeof(msg), "OK;PID;%s\n", pid);
            } else {
                snprintf(msg, sizeof(msg), "KO;PID;%s;UNKNOWN\n", pid);
            }
            USB_Comm_Send(msg);
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
        else if (strcmp(token, "FREQUENCY")) {
            uint32_t freq = atoi(strtok(NULL, ";"));
            Comm_SetPublishFrequency(freq);
            char response[64];
            snprintf(response, sizeof(response), "OK;FREQUENCY;%ld\n", freq);
            USB_Comm_Send(response);
        }
        else {
            char response[268];
            snprintf(response, sizeof(response), "KO;UNKNOWN;%s\n", parse_buffer);
            USB_Comm_Send(response);
        }
    }
    else {
        char response[268];
        snprintf(response, sizeof(response), "KO;UNKNOWN;%s\n", parse_buffer);
        USB_Comm_Send(response);
    }
}
