#ifndef Laser_L1_DMA_H
#define Laser_L1_DMA_H

#define CommandBufferSize 5
#define ResponseBufferSize 8

void Laser_ReceiveDMA_Start(void);
void Laser_TransmitDMA(void);

#endif /* Laser_L1_DMA_H */
