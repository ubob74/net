#ifndef _EMAC_H_
#define _EMAC_H_

struct platform_device;

/* CCU */
#define CCU_BASE_ADDR			0x01C20000
#define CCU_SIZE				0x400
#define CLK_GATING_REG0			0x60
#define CLK_GATING_REG4			0x70
#define SW_RST_REG0				0x02C0
#define SW_RST_REG2				0x02C8

#define EMAC_GATING				17 /* Gating clock for EMAC, 0 - mask, 1 - pass */
#define EMAC_RST				17 /* EMAC reset, 0 - assert, 1 -deassert */

#define EPHY_GATING				0 /* EPHY gating, 0 - assert, 1 - deassert */
#define EPHY_RST				2 /* EPHY reset, 0 - assert, 1 - deassert */

/* SYSCON */
#define SYSCON_BASE_ADDR		0x01C00000
#define SYSCON_SIZE				0x1000
#define SYSCON_EMAC_CLK_REG		0x30

/* EMAC */
#define EMAC_BASE_ADDR			0x01C30000
#define EMAC_SIZE				0x10000

#define EMAC_CTL0				0x00
#define EMAC_CTL1				0x04
#define EMAC_INT_STA			0x08
#define EMAC_INT_EN				0x0C
#define EMAC_TX_CTL0			0x10
#define EMAC_TX_CTL1			0x14
#define EMAC_TX_DMA_LIST		0x20
#define EMAC_RX_CTL0			0x24
#define EMAC_RX_CTL1			0x28
#define EMAC_RX_DMA_LIST		0x34
#define EMAC_RX_FRM_FLT			0x38
#define MII_CMD					0x48
#define MII_DATA				0x4C
#define EMAC_ADDR_HIGH			0x50
#define EMAC_ADDR_LOW			0x54
#define EMAC_TX_DMA_CUR_DESC	0xB4
#define EMAC_TX_DMA_CUR_BUF		0xB8
#define EMAC_RX_DMA_CUR_DESC	0xC4
#define EMAC_RX_DMA_CUR_BUF		0xC8

/* EMAC_INT_EN bits */
#define EMAC_RX_INT				BIT(8)
#define EMAC_BUF_UA_INT_EN		BIT(2)
#define EMAC_TX_INT				BIT(0)

/* EMAC MII_CMD bits */
#define MII_PHY_ADDR_SHIFT		12
#define MII_PHY_REG_ADDR_SHIFT	4
#define MII_BUSY				BIT(0)
#define MII_WRITE				BIT(1)

#define FRAME_LEN_MASK			GENMASK(29, 16)
#define FRAME_LEN_SHIFT			16

#define TX_BUF_SIZE_MASK		GENMASK(10, 0)

extern struct platform_device *pdev;
extern void __iomem *emac_base_addr;
extern void __iomem *syscon;
extern void __iomem *ccu_base_addr;

#endif
