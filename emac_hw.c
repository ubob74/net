#include <linux/kernel.h>
#include <linux/io.h>

#include "emac.h"
#include "emac_hw.h"

void emac_set_rx_filter(void)
{
#if 0
	u32 v = readl(emac_base_addr + EMAC_RX_FRM_FLT);
	/* Receive all frames */
#endif
	u32 v = 1;
	writel(v, emac_base_addr + EMAC_RX_FRM_FLT);
}

void emac_enable_irq(void)
{
	//writel(EMAC_RX_INT | EMAC_TX_INT | EMAC_BUF_UA_INT_EN,
	writel(EMAC_RX_INT | EMAC_TX_INT,
		emac_base_addr + EMAC_INT_EN);
}

void emac_disable_rx_irq(void)
{
	u32 val = readl(emac_base_addr + EMAC_INT_EN);
	val &= ~EMAC_RX_INT;
	writel(val, emac_base_addr + EMAC_INT_EN);
}

void emac_enable_rx_irq(void)
{
	u32 val = readl(emac_base_addr + EMAC_INT_EN);
	val |= EMAC_RX_INT;
	writel(val, emac_base_addr + EMAC_INT_EN);
}

void emac_enable_rx(void)
{
	u32 v = readl(emac_base_addr + EMAC_RX_CTL0);
	v |= BIT(31);
	writel(v, emac_base_addr + EMAC_RX_CTL0);
}

void emac_enable_tx(void)
{
	u32 v = readl(emac_base_addr + EMAC_TX_CTL0);
	v |= BIT(31);
	writel(v, emac_base_addr + EMAC_TX_CTL0);
}

void emac_start_rx(void)
{
	u32 v = readl(emac_base_addr + EMAC_RX_CTL1);
	v |= BIT(30);
	v |= BIT(31);
	writel(v, emac_base_addr + EMAC_RX_CTL1);
}

void emac_stop_rx(void)
{
	u32 v = readl(emac_base_addr + EMAC_RX_CTL1);
	v &= ~BIT(30);
	writel(v, emac_base_addr + EMAC_RX_CTL1);
}

void emac_start_tx(void)
{
	u32 v = readl(emac_base_addr + EMAC_TX_CTL1);
	v |= BIT(30);
	v |= BIT(31);
	writel(v, emac_base_addr + EMAC_TX_CTL1);
}

void emac_stop_tx(void)
{
	u32 v = readl(emac_base_addr + EMAC_TX_CTL1);
	v &= ~BIT(30);
	writel(v, emac_base_addr + EMAC_TX_CTL1);
}

void emac_hw_init(void)
{
	writel(0, emac_base_addr + EMAC_RX_CTL1);
	writel(0, emac_base_addr + EMAC_TX_CTL1);
	writel(0, emac_base_addr + EMAC_RX_FRM_FLT);
	writel(0, emac_base_addr + EMAC_RX_DMA_LIST);
	writel(0, emac_base_addr + EMAC_TX_DMA_LIST);
	writel(0, emac_base_addr + EMAC_INT_EN);
	/* Clear interrupt state */
	writel(0x1FFFFFF, emac_base_addr + EMAC_INT_STA);
}

void emac_set_dma_desc_rx_list(unsigned long addr)
{
	writel(addr, emac_base_addr + EMAC_RX_DMA_LIST);
}

void emac_set_dma_desc_tx_list(unsigned long addr)
{
	writel(addr, emac_base_addr + EMAC_TX_DMA_LIST);
}

void emac_set_mac_addr(u8 *addr)
{
	u32 data;

	data = (addr[5] << 8) | addr[4];
	//data |= BIT(31);
	writel(data, emac_base_addr + EMAC_ADDR_HIGH);

	data = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0];
	writel(data, emac_base_addr + EMAC_ADDR_LOW);
}

void emac_get_mac_addr(u8 *addr)
{
	u32 addr_high, addr_low;

	/* Read MAC address from hardware */
	addr_high = readl(emac_base_addr + EMAC_ADDR_HIGH);
	addr_low = readl(emac_base_addr + EMAC_ADDR_LOW);

	addr[0] = addr_low & 0xff;
	addr[1] = (addr_low >> 8) & 0xff;
	addr[2] = (addr_low >> 16) & 0xff;
	addr[3] = (addr_low >> 24) & 0xff;
	addr[4] = addr_high & 0xff;
	addr[5] = (addr_high >> 8) & 0xff;
}

void emac_set_tx_operation_mode(void)
{
	unsigned long v = readl(emac_base_addr + EMAC_TX_CTL1);
	/* TX_MD Transmission starts after full frame located in TX DMA FIFO */
	v |= BIT(1);
	writel(v, emac_base_addr + EMAC_TX_CTL1);
}

void emac_set_rx_operation_mode(void)
{
	unsigned long v = readl(emac_base_addr + EMAC_RX_CTL1);
	/* RX_MD RX DMA reads data from RX DMA FIFO to host memory
	 after a complete frame has been written to RX DMA FIFO */
	v |= BIT(1);
	//v |= BIT(2);
	writel(v, emac_base_addr + EMAC_RX_CTL1);
}
