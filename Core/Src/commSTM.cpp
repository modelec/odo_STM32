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
            float p, i, d, v_min, v_max;
            if (Comm_GetPID(pid, p, i, d, v_min, v_max)) {
                char response[64];
                snprintf(response, sizeof(response), "SET;PID;%.4f;%.4f;%.4f;%.4f;%.4f\n", p, i, d, v_min, v_max);
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
        else if (strcmp(token, "FREQUENCY") == 0) {
            token = strtok(NULL, ";");
            if (!token) return;
            uint32_t freq;
            Comm_GetPublishFrequency(freq);
            char response[64];
            snprintf(response, sizeof(response), "SET;FREQUENCY;%ld\n", freq);
            USB_Comm_Send(response);
        }
        else if (strcmp(token, "WAYPOINT") == 0) {
            token = strtok(NULL, ";");
            if (!token) return;

            float x, y, theta;
            uint8_t id, type;
            bool active;

            if (strcmp(token, "ACTIVE") == 0) {
            	Comm_GetActiveWaypoint(id, type, x, y, theta, active);
            }
            else {
            	id = atoi(token);
            	Comm_GetWaypoint(id, type, x, y, theta, active);
            }

            char response[128];
            snprintf(response, sizeof(response), "SET;WAYPOINT;%d;%d;%.4f;%.4f;%.4f;%d\n", id, type, x, y, theta, active);
            USB_Comm_Send(response);
        }
        else if (strcmp(token, "MOTOR") == 0) {
        	float l, r;
        	Comm_GetPWM(l, r);
            char response[64];
            snprintf(response, sizeof(response), "SET;MOTOR;%.2f;%.2f\n", l, r);
            USB_Comm_Send(response);
        }
        else if (strcmp(token, "ACTION") == 0) {
        	uint8_t action = Comm_GetAction();
            char response[64];
            snprintf(response, sizeof(response), "SET;ACTION;%d\n", action);
            USB_Comm_Send(response);
        }
        else if (strcmp(token, "PRECISE") == 0) {
            token = strtok(NULL, ";");

            if (!token) {
            	USB_Comm_Send("KO;UNKNOWN;NEED_DATA");
            }
            else {
            	float d;

				if (strcmp(token, "POS") == 0) {
					token = strtok(NULL, ";");

					if (strcmp(token, "FINAL") == 0) {
						d = Comm_GetPrecisePosFinal();
			            char response[64];
			            snprintf(response, sizeof(response), "SET;PRECISE;POS;FINAL;%.4f\n", d);
			            USB_Comm_Send(response);
					}
					else {
						d = Comm_GetPrecisePos();
			            char response[64];
			            snprintf(response, sizeof(response), "SET;PRECISE;POS;%.4f\n", d);
			            USB_Comm_Send(response);
					}
				}
				else if (strcmp(token, "ANGLE") == 0) {
					d = Comm_GetPreciseAngle();
		            char response[64];
		            snprintf(response, sizeof(response), "SET;PRECISE;ANGLE;%.4f\n", d);
		            USB_Comm_Send(response);
				}
            }
        }
        else if (strcmp(token, "DATA") == 0) {
        	token = strtok(NULL, ";");

        	if (token) {
        		if (strcmp(token, "NO") == 0) {
                	token = strtok(NULL, ";");
					if (strcmp(token, "MOVE") == 0) {
						uint32_t time = Comm_GetNotMoveTime();
			            char response[64];
			            snprintf(response, sizeof(response), "SET;NOT;MOVE;%ld\n", time);
			            USB_Comm_Send(response);
                	}
        		}
        	}
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

            char msg[64];
            snprintf(msg, sizeof(msg), "OK;START;%d\n", val != 0);
        	USB_Comm_Send(msg);
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
        else if (strcmp(token, "FREQUENCY") == 0) {
            uint32_t freq = atoi(strtok(NULL, ";"));
            Comm_SetPublishFrequency(freq);
            char response[64];
            snprintf(response, sizeof(response), "OK;FREQUENCY;%ld\n", freq);
            USB_Comm_Send(response);
        }
        else if (strcmp(token, "ACTION") == 0) {
            uint8_t action = atoi(strtok(NULL, ";"));
            Comm_SetAction(action);
            char response[64];
            snprintf(response, sizeof(response), "OK;ACTION;%d\n", action);
            USB_Comm_Send(response);
        }
        else if (strcmp(token, "ALIGNMENT") == 0) {
        	token = strtok(NULL, ";");

            uint8_t action;

            if (strcmp(token, "LEFT") == 0) {
            	action = 1;
        	}
        	else if (strcmp(token, "TOP") == 0) {
            	action = 2;
        	}
        	else if (strcmp(token, "RIGHT") == 0) {
            	action = 3;
			}
        	else if (strcmp(token, "BOTTOM") == 0) {
            	action = 4;
			}

    		Comm_SetAlignment(action);

            char response[64];
            snprintf(response, sizeof(response), "OK;ACTION;%d\n", action);
            USB_Comm_Send(response);
        }
        else if (strcmp(token, "PRECISE") == 0) {
            token = strtok(NULL, ";");

            if (!token) {
            	USB_Comm_Send("KO;UNKNOWN;NEED_DATA");
            }
            else {
				if (strcmp(token, "POS") == 0) {
					token = strtok(NULL, ";");

					if (strcmp(token, "FINAL") == 0) {
						float d = atof(strtok(NULL, ";"));

						Comm_SetPrecisePosFinal(d);
			            char response[64];
			            snprintf(response, sizeof(response), "SET;PRECISE;POS;FINAL;%.4f\n", d);
			            USB_Comm_Send(response);
					}
					else {
						float d = atof(token);

						Comm_SetPrecisePos(d);
			            char response[64];
			            snprintf(response, sizeof(response), "SET;PRECISE;POS;%.4f\n", d);
			            USB_Comm_Send(response);
					}
				}
				else if (strcmp(token, "ANGLE") == 0) {
					float d = atof(strtok(NULL, ";"));

					Comm_SetPreciseAngle(d);
		            char response[64];
		            snprintf(response, sizeof(response), "SET;PRECISE;ANGLE;%.4f\n", d);
		            USB_Comm_Send(response);
				}
            }
        }
        else if (strcmp(token, "DATA") == 0) {
        	token = strtok(NULL, ";");

        	if (token) {
        		if (strcmp(token, "NO") == 0) {
                	token = strtok(NULL, ";");
					if (strcmp(token, "MOVE") == 0) {
						token = strtok(NULL, ";");

						uint32_t time = strtoul(token, NULL, 10);
						Comm_SetNotMoveTime(time);
			            char response[64];
			            snprintf(response, sizeof(response), "SET;NOT;MOVE;%ld\n", time);
			            USB_Comm_Send(response);
                	}
        		}
        	}
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
