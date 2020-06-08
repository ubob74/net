#ifndef _EMAC_HW_H_
#define _EMAC_HW_H_

void emac_set_rx_filter(void);
void emac_enable_irq(void);
void emac_enable_rx_irq(void);
void emac_disable_irq(void);
void emac_disable_rx_irq(void);
void emac_enable_rx(void);
void emac_enable_tx(void);
void emac_start_rx(void);
void emac_stop_rx(void);
void emac_start_tx(void);
void emac_stop_tx(void);
void emac_hw_init(void);
void emac_set_dma_desc_tx_list(unsigned long);
void emac_set_dma_desc_rx_list(unsigned long);
void emac_set_mac_addr(u8 *addr);
void emac_get_mac_addr(u8 *addr);
void emac_set_tx_operation_mode(void);
void emac_set_rx_operation_mode(void);

#endif
