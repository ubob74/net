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
#include <linux/phy.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/of.h>

#include "phy.h"
#include "emac.h"
#include "emac_hw.h"

#define RX_SIZE				4
#define TX_SIZE				4
#define DMA_BUF_SIZE		0x7FF /* 2047 */

#define STMMAC_RESOURCE_NAME "stmmaceth"

struct platform_device *pdev;
void __iomem *emac_base_addr;
void __iomem *syscon;
void __iomem *ccu_base_addr;
static int emac_irq;

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

	/* PHY stuff */
	struct mii_bus *mii_bus;
	struct phy_device *phydev;
	int phy_addr;
	unsigned int link;
	unsigned int speed;
	unsigned int duplex;
};

static int emac_start(struct net_device *ndev);
static int emac_stop(struct net_device *ndev);
static int emac_mdio_init(struct net_device *ndev);

static struct clk *emac_clk = NULL;
static struct reset_control *emac_rst = NULL;

static struct clk *ephy_clk;
static struct reset_control *ephy_rst;

static inline void syscon_setup(void)
{
	syscon = devm_ioremap(&pdev->dev, SYSCON_BASE_ADDR, SYSCON_SIZE);
	BUG_ON(!syscon);
}

static inline void ccu_setup(void)
{
	ccu_base_addr = devm_ioremap(&pdev->dev, CCU_BASE_ADDR, CCU_SIZE);
	BUG_ON(!ccu_base_addr);
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

static int get_ephy_nodes(struct device_node *np)
{
	struct device_node *mii_phy;

	mii_phy = of_get_child_by_name(np, "mii-phy");
	if (!mii_phy) {
		dev_info(&pdev->dev, "Couldn't find node 'mii-phy'\n");
		return -ENODEV;
	}

	ephy_clk = of_clk_get(mii_phy, 0);
	if (IS_ERR(ephy_clk)) {
		dev_info(&pdev->dev, "Can't get ephy clk\n");
		return -ENODEV;
	}

	ephy_rst = of_reset_control_get_exclusive(mii_phy, NULL);
	if (IS_ERR(ephy_rst)) {
		dev_info(&pdev->dev, "Can't get ephy reset\n");
		return -ENODEV;
	}

	dev_info(&pdev->dev, "Found internal mii nodes\n");

	of_node_put(mii_phy);

	return 0;
}

static int emac_ephy_hw_setup(void)
{
	int ret;
	unsigned int val;
	struct device *dev = &pdev->dev;

	ret = get_ephy_nodes(dev->of_node);
	if (ret)
		return ret;

	clk_prepare_enable(ephy_clk);

	reset_control_reset(ephy_rst);

	val = readl(syscon + SYSCON_EMAC_EPHY_CLK_REG);
	dev_info(dev, "syscon=%lx val=%x\n", (unsigned long)syscon, val);

	/* Setup PHY address */
	val |= (PHY_ADDR & 0x1F) << 20;

	/* Select internal PHY */
	val |= BIT(15);

	/* Power up PHY */
	if (val & BIT(16))
		val &= ~BIT(16);

	dev_info(&pdev->dev, "syscon val=%x\n", val);

	writel(val, syscon + SYSCON_EMAC_EPHY_CLK_REG);

	return 0;
}

static int emac_hw_setup(void)
{
	int ret;
	struct resource *res;
	struct device *dev = &pdev->dev;

	emac_clk = devm_clk_get(dev, STMMAC_RESOURCE_NAME);
	if (IS_ERR(emac_clk)) {
		dev_warn(dev, "Cannot get clock\n");
		emac_clk = NULL;
		return -1;
	}

	clk_prepare_enable(emac_clk);

	emac_rst = devm_reset_control_get(dev, STMMAC_RESOURCE_NAME);
	if (IS_ERR(emac_rst)) {
		dev_warn(dev, "Cannot get reset control\n");
		clk_disable_unprepare(emac_clk);
		return -1;
	}

	ret = reset_control_assert(emac_rst);
	reset_control_deassert(emac_rst);
	if (ret == -ENOTSUPP)
	    reset_control_reset(emac_rst);

	emac_irq = platform_get_irq_byname(pdev, "macirq");
	if (emac_irq < 0) {
		dev_err(dev, "Can't get irq\n");
		return -EFAULT;
	} else {
		dev_dbg(dev, "emac_irq=%d\n", emac_irq);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	emac_base_addr = devm_ioremap_resource(&pdev->dev, res);

	dev_info(dev, "emac_base_addr=%08x\n", (u32)emac_base_addr);

	return 0;
}

static u32 emac_tx_avail(struct emac_priv *priv)
{
	struct tx_queue *tx_q = &priv->tx_q;
	u32 avail;

	if (tx_q->dirty_tx > tx_q->cur_tx)
		avail = tx_q->dirty_tx - tx_q->cur_tx - 1;
	else
		avail = TX_SIZE - tx_q->cur_tx + tx_q->dirty_tx - 1;

	return avail;
}

static void emac_tx_clean(struct net_device *ndev)
{
	struct emac_priv *priv = netdev_priv(ndev);
	struct tx_queue *tx_q = &priv->tx_q;
	u32 entry;

	entry = tx_q->dirty_tx;
	while (entry != tx_q->cur_tx) {
		struct sk_buff *skb = tx_q->sk_buff[entry];
		struct dma_desc *p = tx_q->dma_tx + entry;

		if (p->status & BIT(31))
			break;

		if (tx_q->sk_buff_dma[entry]) {
			dma_unmap_single(priv->dev,
				tx_q->sk_buff_dma[entry],
				tx_q->len,
				DMA_TO_DEVICE
			);
		}

		if (skb != NULL) {
			dev_consume_skb_any(skb);
			tx_q->sk_buff[entry] = NULL;
		}

		entry = (entry + 1) % TX_SIZE;
	}
	tx_q->dirty_tx = entry;

	if (netif_queue_stopped(ndev) && (emac_tx_avail(priv) > 1)) {
		dev_dbg(priv->dev, "%s: restarting queue\n", __func__);
		netif_wake_queue(ndev);
	}
}

static netdev_tx_t emac_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct emac_priv *priv = netdev_priv(ndev);
	struct dma_desc *desc;
	struct tx_queue *tx_q;
	int entry = 0;
	unsigned int len = skb_headlen(skb);
	int nfrags = skb_shinfo(skb)->nr_frags;
	dma_addr_t dma_phy;
	u32 avail;

	tx_q = &priv->tx_q;

	dev_dbg(priv->dev, "%s: enter, nfrags=%d len=%u\n", __func__, nfrags, len);

	entry = tx_q->cur_tx;
	desc = tx_q->dma_tx + entry;
	tx_q->cur_tx = (tx_q->cur_tx + 1) % TX_SIZE;

	tx_q->sk_buff[entry] = skb;

	avail = emac_tx_avail(priv);
	if (avail <= (MAX_SKB_FRAGS + 1))
		netif_stop_queue(ndev);

	skb_tx_timestamp(skb);
	tx_q->len = len;

	dma_phy = dma_map_single(priv->dev,
		skb->data,
		len,
		DMA_TO_DEVICE);
	if (dma_mapping_error(priv->dev, dma_phy))
		dev_err(priv->dev, "dma_map_single error\n");

	tx_q->sk_buff_dma[entry] = dma_phy;
	desc->buf_addr = dma_phy;

	desc->st = cpu_to_le32(len & TX_BUF_SIZE_MASK);
	desc->st |= BIT(31); /* TX_INT in ISR when current frame have been transmitted */
	desc->st |= BIT(30) | BIT(29); /* first and last segment */
	/*desc->st |= BIT(24);*/
	desc->status |= BIT(31);

	wmb();

	emac_start_tx();

	return NETDEV_TX_OK;
}

static int emac_rx(struct net_device *ndev)
{
	struct emac_priv *priv = netdev_priv(ndev);
	struct rx_queue *rx_q = &priv->rx_q;
	unsigned int entry = rx_q->cur_rx;
	unsigned int next_entry;
	unsigned int count = 0;
	struct dma_desc *p; // = rx_q->dma_rx + entry;
	int status;
	struct sk_buff *skb;
	int frame_len;

	while (1) {
		p = rx_q->dma_rx + entry;
		status = p->status;

		/* check if managed by the DMA */
		if (status & BIT(31))
			break;

		count++;

		frame_len = (status & FRAME_LEN_MASK) >> FRAME_LEN_SHIFT;
		if (frame_len > 1536)
			break;

		skb = netdev_alloc_skb(priv->ndev, frame_len);

		dma_sync_single_for_cpu(priv->dev,
				rx_q->sk_buff_dma[entry],
				frame_len,
				DMA_FROM_DEVICE);
		skb_copy_to_linear_data(skb,
				rx_q->sk_buff[entry]->data,
				frame_len);
		skb_put(skb, frame_len);
		dma_sync_single_for_device(priv->dev,	
				rx_q->sk_buff_dma[entry],
				frame_len,
				DMA_FROM_DEVICE);

		skb->protocol = eth_type_trans(skb, priv->ndev);

		netif_rx(skb);

		rx_q->cur_rx = (rx_q->cur_rx + 1) % RX_SIZE;
		next_entry = rx_q->cur_rx;

		entry = next_entry;
		p->status |= BIT(31);
	}

	return count;
}

static int emac_set_mac_address(struct net_device *ndev, void *addr)
{
	int ret;
	struct emac_priv *priv = netdev_priv(ndev);
	struct device *dev = priv->dev;

    dev_info(dev, "Set MAC address: %s\n", __func__);

    ret = eth_mac_addr(ndev, addr);
	if (ret)
		return ret;

	emac_set_mac_addr(ndev->dev_addr);

	return ret;
}

static struct net_device_ops emac_netdev_ops = {
	.ndo_open = emac_start,
	.ndo_start_xmit = emac_xmit,
	.ndo_stop = emac_stop,
	.ndo_set_mac_address = emac_set_mac_address,
};

static int emac_net_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev = NULL;
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
	priv->rx_q.priv = priv;
	(&priv->tx_q)->priv = priv;

	ndev->netdev_ops = &emac_netdev_ops;

    eth_hw_addr_random(ndev);

	dev_set_drvdata(dev, priv->ndev);

	if (emac_mdio_init(ndev)) {
		dev_err(dev, "MII init error\n");
		return -1;
	}

	return register_netdev(ndev);
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
		dma_phy += sizeof(struct dma_desc);
		p->next = dma_phy;
	}
	p->next = rx_q->dma_rx_phy;

	rx_q->cur_rx = 0;

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

static int emac_tx_queue_init(struct net_device *ndev)
{
	int i;
	struct emac_priv *priv = netdev_priv(ndev);
	struct device *dev = priv->dev;
	struct tx_queue *tx_q = &priv->tx_q;
	struct dma_desc *p;
	dma_addr_t dma_phy;

	tx_q->sk_buff = kmalloc_array(TX_SIZE, sizeof(struct sk_buff *),
			GFP_KERNEL);

	tx_q->sk_buff_dma = kmalloc_array(TX_SIZE, sizeof(dma_addr_t),
			GFP_KERNEL);

	tx_q->dma_tx = dma_alloc_coherent(dev,
			TX_SIZE * sizeof(struct dma_desc),
			&tx_q->dma_tx_phy,
			GFP_KERNEL);

	memset(tx_q->dma_tx, 0, TX_SIZE * sizeof(struct dma_desc));

	dev_info(dev, "tx_q: tx_q=%08lx\n", (unsigned long)tx_q);
	dev_info(dev, "tx_q: sk_buff=%08x sk_buff_dma=%08x\n",
			(u32)tx_q->sk_buff, (u32)tx_q->sk_buff_dma);
	dev_info(dev, "tx_q: dma_tx=%08x dma_tx_phy=%08x\n",
			(u32)tx_q->dma_tx, (u32)tx_q->dma_tx_phy);

	/* Init TX DMA chain */
	dma_phy = tx_q->dma_tx_phy;
	for (i = 0; i < TX_SIZE; i++) {
		p = tx_q->dma_tx + i;
		/*p->st |= BIT(24);*/
		dma_phy += sizeof(struct dma_desc);
		p->next = dma_phy;
	}
	p->next = tx_q->dma_tx_phy;

	tx_q->cur_tx = 0;
	tx_q->dirty_tx = 0;

	/* Setup transmit DMA descriptor list */
	emac_set_dma_desc_tx_list(tx_q->dma_tx_phy);

	return 0;
}

static void emac_tx_queue_release(struct net_device *ndev)
{
	struct emac_priv *priv = netdev_priv(ndev);
	struct tx_queue *tx_q = &priv->tx_q;

	dma_free_coherent(priv->dev, TX_SIZE * sizeof(struct dma_desc),
			tx_q->dma_tx, tx_q->dma_tx_phy);

	kfree(tx_q->sk_buff_dma);
	kfree(tx_q->sk_buff);
}

static irqreturn_t emac_interrupt(int irq, void *dev_id)
{
	int status;
	struct net_device *ndev = (struct net_device *)dev_id;
	struct emac_priv *priv = netdev_priv(ndev);
	struct device *dev = priv->dev;

	if (!dev)
		return -EINVAL;

	status = readl(emac_base_addr + EMAC_INT_STA);
	writel(status, emac_base_addr + EMAC_INT_STA);
	dev_dbg(dev, "Int status=%08x\n", status);

	if (status & EMAC_RX_INT)
		emac_rx(ndev);

	if (status & EMAC_TX_INT)
		emac_tx_clean(ndev);

	return IRQ_HANDLED;
}

static int emac_phy_read(struct mii_bus *mii_bus, int phy_addr, int phy_reg)
{
	int err;
	u32 v, val = MII_BUSY;
	int data;

	val |= (phy_addr << MII_PHY_ADDR_SHIFT );
	val |= (phy_reg << MII_PHY_REG_ADDR_SHIFT);
	val |= (3 << 20);

	err = readl_poll_timeout(emac_base_addr + MII_CMD, v,
		!(v & MII_BUSY), 100, 100000);
	if (err)
		return -EBUSY;

	writel(0, emac_base_addr + MII_DATA);
	writel(val, emac_base_addr + MII_CMD);

	err = readl_poll_timeout(emac_base_addr + MII_CMD, val,
		!(val & MII_BUSY), 100, 100000);
	if (err) {
		dev_err(&pdev->dev, "PHY read timeout\n");
		return -EBUSY;
	}

	data = (int)readl(emac_base_addr + MII_DATA);
	return data;
}

static int emac_phy_write(struct mii_bus *mii_bus, int phy_addr, int phy_reg, u16 data)
{
	int err;
	u32 v, val = MII_BUSY;

	val |= (phy_addr << MII_PHY_ADDR_SHIFT);
	val |= (phy_reg << MII_PHY_REG_ADDR_SHIFT) | MII_WRITE;
	val |= (3 << 20);

	/* Wait until any existing MII operation is complete */
	if (readl_poll_timeout(emac_base_addr + MII_CMD, v,
			!(v & MII_BUSY), 100, 100000))
		return -EBUSY;

	writel(data, emac_base_addr + MII_DATA);
	writel(val, emac_base_addr + MII_CMD);

	err = readl_poll_timeout(emac_base_addr + MII_CMD, val,
		!(val & MII_BUSY), 100, 100000);
	if (err) {
		dev_err(&pdev->dev, "PHY write timeout\n");
		return -EFAULT;
	}

	return 0;
}

static void emac_phylink_handler(struct net_device *ndev)
{
	int v;
	struct emac_priv *priv = netdev_priv(ndev);
	struct phy_device *phydev = priv->phydev;
	int duplex, speed;

	phy_print_status(phydev);

	netdev_info(ndev, "duplex %s, speed %s\n",
		phy_duplex_to_str(phydev->duplex), phy_speed_to_str(phydev->speed));

	switch (phydev->speed) {
	case SPEED_10:
		speed = SPEED10;
		break;
	case SPEED_100:
		speed = SPEED100;
		break;
	default:
		break;
	}

	duplex = phydev->duplex;

	v = (speed << 2) | duplex;
	writel(v, emac_base_addr + EMAC_CTL0);
	v = readl(emac_base_addr + EMAC_CTL0);
	netdev_info(ndev, "EMAC_CTL0=%08x\n", v);
}

static int emac_mdio_init(struct net_device *ndev)
{
	int addr, err;
	struct emac_priv *priv = netdev_priv(ndev);
	struct mii_bus *mii_bus;
	struct phy_device *phydev;

	priv->mii_bus = mdiobus_alloc();
	if (!priv->mii_bus) {
		dev_err(priv->dev, "mdiobus_alloc error\n");
		return -ENOMEM;
	}

	mii_bus = priv->mii_bus;

	mii_bus->name = "emac-mdio";
	mii_bus->read = emac_phy_read;
	mii_bus->write = emac_phy_write;

	snprintf(mii_bus->id, MII_BUS_ID_SIZE, "%s", mii_bus->name);

	mii_bus->priv = priv;

	if (mdiobus_register(mii_bus)) {
		err = -ENXIO;
		goto err_out;
	}

	addr = PHY_ADDR;
	phydev = mdiobus_get_phy(mii_bus, addr);
	if (!phydev) {
		pr_err("address error\n");
		goto err_no_phy;
	}

	priv->phy_addr = addr;
	priv->phydev = phydev;

	return 0;

err_no_phy:
	mdiobus_unregister(mii_bus);
err_out:
	mdiobus_free(mii_bus);
	return err;
}

static int emac_start(struct net_device *ndev)
{
	int v, ret;
	u8 addr[6];
	struct emac_priv *priv = netdev_priv(ndev);
	struct device *dev = priv->dev;
	struct phy_device *phydev = priv->phydev;

	/* Allocate RX and TX queues */
	emac_rx_queue_init(ndev);
	emac_tx_queue_init(ndev);

	/* HW address setup */
	emac_get_mac_addr(addr);
	dev_info(&ndev->dev, "%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
		addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
	emac_set_mac_addr(ndev->dev_addr);
	emac_get_mac_addr(addr);
	dev_info(&ndev->dev, "%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
		addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

	emac_set_tx_operation_mode();
	emac_set_rx_operation_mode();

	/* Request the IRQ line */
	ret = request_irq(emac_irq, emac_interrupt, IRQF_SHARED, "emac-irq", ndev);
	if (unlikely(ret < 0)) {
		dev_err(dev, "error allocating IRQ %d, error=%d\n", emac_irq, ret);
		return -EINVAL;
	}

	ret = phy_connect_direct(ndev, phydev, emac_phylink_handler, PHY_INTERFACE_MODE_MII);
	if (ret) {
		netdev_err(ndev, "Couldn't connect phy\n");

		free_irq(priv->emac_irq, ndev);
		emac_rx_queue_release(ndev);
		emac_tx_queue_release(ndev);

		goto err_no_phy;
	}

	phy_attached_info(phydev);
	phy_start(phydev);

	/* Read frame control status */
	v = readl(emac_base_addr + 0x1C);
	dev_info(dev, "Frame control: v=%08x\n", v);

	/* Setup receive filter */
	emac_set_rx_filter();

	/* Enable RX */
	emac_enable_rx();

	/* Enable TX */
	emac_enable_tx();

	/* Start RX DMA */
	emac_start_rx();

	/* Start TX DMA */
	emac_start_tx();

	/* Enable interrupts */
	emac_enable_irq();

err_no_phy:
	return ret;
}

static int emac_stop(struct net_device *ndev)
{
	struct emac_priv *priv = netdev_priv(ndev);
	struct phy_device *phydev = priv->phydev;

    dev_info(&ndev->dev, "Stopping emac..\n");

	emac_stop_tx();
	emac_stop_rx();

    phy_stop(phydev);
    phy_disconnect(phydev);

	free_irq(priv->emac_irq, ndev);

	emac_rx_queue_release(ndev);
	emac_tx_queue_release(ndev);

	return 0;
}

static int emac_release(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct emac_priv *priv = netdev_priv(ndev);
	struct mii_bus *mii_bus = priv->mii_bus;

	unregister_netdev(ndev);
	free_netdev(ndev);

	mdiobus_unregister(mii_bus);
	mdiobus_free(mii_bus);

    reset_control_put(ephy_rst);
    clk_put(ephy_clk);

	return 0;
}

static int net_probe(struct platform_device *_pdev)
{
	int ret;

	pdev = _pdev;

	dev_info(&pdev->dev, "CCU setup..\n");
	ccu_setup();

	dev_info(&pdev->dev, "Syscon setup..\n");
	syscon_setup();

	dev_info(&pdev->dev, "PHY setup..\n");
	emac_ephy_hw_setup();

	dev_info(&pdev->dev, "EMAC hardware setup..\n");
	ret = emac_hw_setup();
	if (ret)
		return ret;

	emac_net_init(pdev);
	ccu_dump();

	emac_soft_reset(pdev);
	emac_hw_init();

	return 0;
}

static void net_remove(struct platform_device *pdev)
{
	emac_release(pdev);
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
		.name       = "net",
		.of_match_table = net_match,
	},
};
module_platform_driver(net_driver);

MODULE_LICENSE("GPL");
