#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/bits.h>
#include <linux/mii.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>

#include "phy.h"
#include "emac.h"
#include "emac_hw.h"

#define RX_SIZE				4
#define TX_SIZE				4
#define DMA_BUF_SIZE		0x7FF /* 2047 */

struct platform_device *pdev;
void __iomem *emac_base_addr;
void __iomem *syscon;
void __iomem *ccu_base_addr;
static int emac_irq = 0;

struct dma_desc {
	u32 status;
	u32 st;
	u32 buf_addr;
	u32 next;
} __attribute__ ((aligned(0x1000)));

struct rx_queue {
	struct emac_priv *priv;
	struct sk_buff **sk_buff;
	dma_addr_t *sk_buff_dma;
	struct dma_desc *dma_rx ____cacheline_aligned_in_smp;
	dma_addr_t dma_rx_phy;
	unsigned int cur_rx;
	struct napi_struct napi;
};

struct tx_queue {
	struct emac_priv *priv;
	struct sk_buff **sk_buff;
	dma_addr_t *sk_buff_dma;
	struct dma_desc *dma_tx ____cacheline_aligned_in_smp;
	dma_addr_t dma_tx_phy;
	unsigned int cur_tx;
	unsigned int dirty_tx;
	unsigned int len;
};

struct ccu_dump_reg {
	char *reg;
	u32 offset;
};

struct emac_priv {
	void __iomem *emac_base_addr;
	int emac_irq;
	struct net_device *ndev;
	struct device *dev;

	struct rx_queue rx_q;
	struct tx_queue tx_q;
};


static int duplex = HALF_DUPLEX;
static int speed = SPEED10;

static u8 umac_addr[] = { 0x02, 0x42, 0x8D, 0x01, 0x14, 0x9B };

static int emac_start(struct net_device *ndev);
static int emac_stop(struct net_device *ndev);

static struct net_device *ndev = NULL;

static inline void syscon_setup(void)
{
	syscon = devm_ioremap(&pdev->dev, SYSCON_BASE_ADDR, SYSCON_SIZE);
}

static inline void ccu_setup(void)
{
	ccu_base_addr = devm_ioremap(&pdev->dev, CCU_BASE_ADDR, CCU_SIZE);
}

static void ccu_dump(void)
{
	int i;
	u32 val;
	struct ccu_dump_reg r[] = {
		{ "clk gating reg4", CLK_GATING_REG4 },
		{ "clk gating reg0", CLK_GATING_REG0 },
		{ "rst reg2", SW_RST_REG2 },
		{ "rst reg0", SW_RST_REG0 },
	};

	pr_cont("ccu_base_addr=%08x ", (u32)ccu_base_addr);
	for (i = 0; i < ARRAY_SIZE(r); i++) {
		val = readl(ccu_base_addr + r[i].offset);
		pr_cont("%s: %08x ", r[i].reg, val);
	}
	pr_cont("\n");
}

static int emac_hw_setup(void)
{
	u32 val;
	struct device *dev = &pdev->dev;

	emac_irq = platform_get_irq_byname(pdev, "macirq");
	if (emac_irq < 0) {
		dev_err(dev, "Can't get irq\n");
		return -EFAULT;
	} else {
		dev_info(dev, "emac_irq=%d\n", emac_irq);
	}

	emac_base_addr = devm_ioremap(&pdev->dev, EMAC_BASE_ADDR, EMAC_SIZE);

	dev_info(dev, "emac_base_addr=%08x\n", (u32)emac_base_addr);

	/* Set clock gating for EMAC - bit 17 in Bus Clock Gating Reg0, offset 0x60 */
	val = readl(ccu_base_addr + CLK_GATING_REG0);
	val |= BIT(EMAC_GATING);
	writel(val, ccu_base_addr + CLK_GATING_REG0);

	/* Reset assert EMAC - bit 17 in Bus Sotfware Reset Reg0, offset 0x2C0 */
	val = readl(ccu_base_addr + SW_RST_REG0);
	val &= ~BIT(EMAC_RST);
	writel(val, ccu_base_addr + SW_RST_REG0);

	/* Reset deassert EMAC */
	val = readl(ccu_base_addr + SW_RST_REG0);
	val |= BIT(EMAC_RST);
	writel(val, ccu_base_addr + SW_RST_REG0);

	val = readl(syscon + SYSCON_EMAC_CLK_REG);
	dev_info(&pdev->dev, "syscon=%lx val=%x\n", (unsigned long)syscon, val);

	/* Setup PHY address */
	val |= (PHY_ADDR & 0x1F) << 20;

	/* Select internal PHY */
	val |= BIT(15);

	/* Power up PHY */
	if (val & BIT(16))
		val &= ~BIT(16);

	dev_info(&pdev->dev, "syscon val=%x\n", val);

	writel(val, syscon + SYSCON_EMAC_CLK_REG);

	return 0;
}

static void dump_skb(struct sk_buff *skb, const char *title, int frame_len)
{
	int i;

	pr_err("%s\n", title);
	for (i = 0; i < 40; i++)
		pr_cont("%.2x ", skb->data[i]);
	pr_cont("%d bytes\n", frame_len);
}

static void desc_list_dump(struct dma_desc *p, int size)
{
	int i;

	for (i = 0; i < size; i++, p++) {
		pr_err("%d: desc=%08lx status=%08x st=%08x buf_addr=%08x next=%08x\n",
			i, (unsigned long)p, p->status, p->st, p->buf_addr, p->next);
	}
}

static int emac_rx(struct emac_priv *priv)
{
	struct rx_queue *rx_q = &priv->rx_q;
	unsigned int entry = rx_q->cur_rx;
	unsigned int count = 0;
	struct dma_desc *p;
	int status;
	int frame_len;

	p = rx_q->dma_rx + entry;
	status = p->status;
	/* check if managed by the DMA */
	if (status & BIT(31)) {
		pr_err("%s: busy\n", __func__);
		goto out;
	}

	count++;

	frame_len = (status & FRAME_LEN_MASK) >> FRAME_LEN_SHIFT;
	if (frame_len > 1536)
		goto out;

	dma_sync_single_for_cpu(priv->dev,
			rx_q->sk_buff_dma[entry],
			frame_len,
			DMA_FROM_DEVICE);
	dump_skb(rx_q->sk_buff[entry], "Received data:", frame_len);
	dma_sync_single_for_device(priv->dev,	
			rx_q->sk_buff_dma[entry],
			frame_len,
			DMA_FROM_DEVICE);

	rx_q->cur_rx = (rx_q->cur_rx + 1) % RX_SIZE;

	p->status |= BIT(31);

out:
	return count;
}

static int emac_net_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct emac_priv *priv;

	ndev = alloc_etherdev(sizeof(struct emac_priv));
	if (!ndev) {
		dev_err(dev, "net_device alloc error\n");
		return -ENOMEM;
	}

	SET_NETDEV_DEV(ndev, dev);

	priv = netdev_priv(ndev);
	priv->ndev = ndev;
	priv->dev = dev;
	priv->emac_irq = emac_irq;
	priv->emac_base_addr = emac_base_addr;
	(&priv->rx_q)->priv = priv;
	(&priv->tx_q)->priv = priv;

	memcpy(ndev->dev_addr, umac_addr, ETH_ALEN);
	dev_set_drvdata(dev, priv->ndev);

	return 0;
}

static int emac_soft_reset(struct platform_device *pdev)
{
	int err;
	u32 val;

	val = readl(emac_base_addr + EMAC_CTL1);

	/* EMAC soft reset */
	writel(val | 0x1, emac_base_addr + EMAC_CTL1);

	err = readl_poll_timeout(emac_base_addr + EMAC_CTL1, val,
			!(val & 0x01), 100, 100000);
	if (err) {
		dev_err(&pdev->dev, "EMAC reset timeout, err=%d\n", err);
		return -EFAULT;
	}
	dev_info(&pdev->dev, "EMAC reset done\n");

	return 0;
}

static int emac_rx_queue_init(struct net_device *ndev)
{
	int i;
	struct dma_desc *p;
	struct sk_buff *skb;
	struct emac_priv *priv = netdev_priv(ndev);
	struct device *dev = priv->dev;
	struct rx_queue *rx_q = &priv->rx_q;
	dma_addr_t dma_phy;

	rx_q->sk_buff = kmalloc_array(RX_SIZE, sizeof(struct sk_buff *),
			GFP_KERNEL);

	rx_q->sk_buff_dma = kmalloc_array(RX_SIZE, sizeof(dma_addr_t),
			GFP_KERNEL);

	rx_q->dma_rx = dma_alloc_coherent(dev,
			RX_SIZE * sizeof(struct dma_desc),
			&rx_q->dma_rx_phy,
			GFP_KERNEL);

	dev_info(dev, "rx_q: sk_buff=%08x sk_buff_dma=%08x\n",
			(u32)rx_q->sk_buff, (u32)rx_q->sk_buff_dma);
	dev_info(dev, "rx_q: dma_rx=%08x dma_rx_phy=%08x\n",
			(u32)rx_q->dma_rx, (u32)rx_q->dma_rx_phy);

	/* Init RX descriptor buffer */
	for (i = 0; i < RX_SIZE; i++) {
		p = rx_q->dma_rx + i;
		skb = netdev_alloc_skb(ndev, DMA_BUF_SIZE);
		rx_q->sk_buff[i] = skb;
		rx_q->sk_buff_dma[i] = dma_map_single(dev,
				skb->data,
				DMA_BUF_SIZE,
				DMA_FROM_DEVICE);
		/* Descriptor set address */
		p->buf_addr = rx_q->sk_buff_dma[i];
		dev_info(dev, "%d: p=%08x skb=%08x skb_dma=%08x\n", i, (u32)p,
			(u32)rx_q->sk_buff[i], (u32)rx_q->sk_buff_dma[i]);
	}

	/* Init RX DMA chain */
	dma_phy = rx_q->dma_rx_phy;
	for (i = 0; i < RX_SIZE; i++) {
		p = rx_q->dma_rx + i;
		p->status |= BIT(31);
		p->st = DMA_BUF_SIZE;
		p->buf_addr = rx_q->sk_buff_dma[i];
		dma_phy += sizeof(struct dma_desc);
		p->next = dma_phy;
	}
	p->next = rx_q->dma_rx_phy;

	rx_q->cur_rx = 0;

	/* Dump addresses */
	desc_list_dump(rx_q->dma_rx, RX_SIZE);

	/* Setup receive DMA descriptor list */
	emac_set_dma_desc_rx_list(rx_q->dma_rx_phy);

	return 0;
}

static void emac_rx_queue_release(struct net_device *ndev)
{
	int i;
	struct device *dev = &ndev->dev;
	struct emac_priv *priv = netdev_priv(ndev);
	struct rx_queue *rx_q = &priv->rx_q;

	for (i = 0; i < RX_SIZE; i++) {
		if (rx_q->sk_buff[i]) {
			dma_unmap_single(dev,
				rx_q->sk_buff_dma[i],
				sizeof(dma_addr_t),
				DMA_FROM_DEVICE);
			dev_kfree_skb_any(rx_q->sk_buff[i]);
		}
		rx_q->sk_buff[i] = NULL;
	}

	dma_free_coherent(dev, RX_SIZE * sizeof(struct dma_desc),
			rx_q->dma_rx, rx_q->dma_rx_phy);

	kfree(rx_q->sk_buff_dma);
	kfree(rx_q->sk_buff);
}

static irqreturn_t emac_interrupt(int irq, void *dev_id)
{
	int status;
	struct net_device *ndev = (struct net_device *)dev_id;
	struct emac_priv *priv = netdev_priv(ndev);

	status = readl(emac_base_addr + EMAC_INT_STA);
	writel(status, emac_base_addr + EMAC_INT_STA);

	if (status & EMAC_RX_INT)
		emac_rx(priv);

	return IRQ_HANDLED;
}

static int emac_start(struct net_device *ndev)
{
	int ctrl;
	int ret, v;
	u8 addr[6];
	struct emac_priv *priv = netdev_priv(ndev);
	struct device *dev = priv->dev;
	const char *duplex_str;
	const char *speed_str;

	/* Allocate RX and TX queues */
	emac_rx_queue_init(ndev);

	/* HW address setup */
	emac_set_mac_addr(ndev->dev_addr);
	emac_get_mac_addr(addr);
	dev_info(dev, "%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
		addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

	emac_set_rx_operation_mode();

	/* Request the IRQ line */
	ret = request_irq(emac_irq, emac_interrupt, IRQF_SHARED, "emac-irq", ndev);
	if (unlikely(ret < 0)) {
		dev_err(dev, "error allocating IRQ %d, error=%d\n", emac_irq, ret);
		return -EINVAL;
	}

	ret = phy_adjust_link(&speed, &duplex);
	if (ret < 0) {
		dev_info(dev, "Can't do auto-negotiation\n");
		ctrl = phy_read(PHY_ADDR, MII_BMCR);
		dev_info(dev, "BMCR=%08x\n", ctrl);
		if (ctrl & BMCR_FULLDPLX) {
			dev_info(dev, "full duplex\n");
			duplex = FULL_DUPLEX;
		} else {
			dev_info(dev, "half duplex\n");
		}
		if (ctrl & BMCR_SPEED100)
			speed = SPEED100;
	} 

	duplex_str = (duplex == FULL_DUPLEX) ? "full" : "half";
	speed_str = (speed == SPEED100) ? "100Mb/s" : "10Mb/s";

	dev_info(dev, "duplex %s, speed %s\n", duplex_str, speed_str);

	v = (speed << 2) | duplex;
	writel(v, emac_base_addr + EMAC_CTL0);
	v = readl(emac_base_addr + EMAC_CTL0);
	dev_info(dev, "EMAC_CTL0=%08x\n", v);

	v = readl(emac_base_addr + 0x1C);
	dev_info(dev, "Frame control: v=%08x\n", v);

	/* Setup receive filter */
	emac_set_rx_filter();

	/* Enable RX */
	emac_enable_rx();

	/* Start RX DMA */
	emac_start_rx();

	/* Enable interrupts */
	emac_enable_irq();

	return ret;
}

static int emac_stop(struct net_device *ndev)
{
	emac_stop_rx();
	return 0;
}

static int emac_release(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct emac_priv *priv = netdev_priv(ndev);

	free_irq(priv->emac_irq, ndev);
	emac_rx_queue_release(ndev);

	free_netdev(ndev);

	return 0;
}

static int net_probe(struct platform_device *_pdev)
{
	int ret;

	pdev = _pdev;

	dev_info(&pdev->dev, "CCU setup..\n");
	ccu_setup();
	ccu_dump();

	dev_info(&pdev->dev, "Syscon setup..\n");
	syscon_setup();
	ccu_dump();

	dev_info(&pdev->dev, "PHY setup..\n");
	phy_setup();
	ccu_dump();

	dev_info(&pdev->dev, "EMAC hardware setup..\n");
	ret = emac_hw_setup();
	if (ret)
		return ret;
	ccu_dump();

	ret = phy_start();
	if (ret)
		return ret;

	emac_net_init(pdev);
	ccu_dump();

	emac_soft_reset(pdev);
	emac_hw_init();

	emac_start(ndev);

	return 0;
}

static int net_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	emac_stop(ndev);
	phy_stop();

	emac_release(pdev);
	phy_release();

	ccu_dump();

	return 0;
}

static const struct of_device_id net_match[] = {
	{ .compatible = "allwinner,sun8i-h3-emac", },
	{}
};
MODULE_DEVICE_TABLE(of, net_match);

static struct platform_driver net_driver = {
	.probe  = net_probe,
	.remove = net_remove,
	.driver = {
		.name       = "net-test",
		.of_match_table = net_match,
	},
};

module_platform_driver(net_driver);

MODULE_LICENSE("GPL");
