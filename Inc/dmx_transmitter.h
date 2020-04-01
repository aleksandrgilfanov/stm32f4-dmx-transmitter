#ifndef DMX_TRANSMITTER_H
#define DMX_TRANSMITTER_H

#define DMX_BREAK	92
#define DMX_MAB		12
#define DMX_SLOT	44
#define DMX_MBS		0

void dmx_send(const uint8_t *slots, uint16_t size);
void dmx_slot(void);
void dmx_reset_sequence(void);

#endif		/* DMX_TRANSMITTER_H */
