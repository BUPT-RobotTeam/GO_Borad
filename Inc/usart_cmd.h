//
// Created by Trisoil on 2023/6/22.
//

#ifndef GO_BOARD_USART_CMD_H
#define GO_BOARD_USART_CMD_H

extern uint8_t interrupted;

void USART6_Rx_Init();
void Usart6_TX_DMA_Init(void);
void Usart_Printf(const char *fmt,...);

#endif //GO_BOARD_USART_CMD_H
