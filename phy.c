#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mii.h>
#include <linux/platform_device.h>

#include "phy.h"
#include "emac.h"

int phy_setup(void)
{
	volatile u32 val;

	/* Set clock gating for EPHY - bit 0 in Bus Clock Gating Reg4, offset 0x70 */
	val = readl(ccu_base_addr + CLK_GATING_REG4);
	val |= BIT(EPHY_GATING);
	writel(val, ccu_base_addr + CLK_GATING_REG4);

	/* Reset assert EPHY - bit 2 in Bus Software Reset Reg2, offset 0x2C8 */
	val = readl(ccu_base_addr + SW_RST_REG2);
	val &= ~BIT(EPHY_RST);
	writel(val, ccu_base_addr + SW_RST_REG2);

	/* Deassert EPHY */
	val = readl(ccu_base_addr + SW_RST_REG2);
	val |= BIT(EPHY_RST);
	writel(val, ccu_base_addr + SW_RST_REG2);

	mdelay(100);

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

void phy_release(void)
{
}

int phy_read(int phy_addr, int phy_reg)
{
	int err;
	u32 v, val = MII_BUSY;
	int data;

	val |= (phy_addr << MII_PHY_ADDR_SHIFT );
	val |= (phy_reg << MII_PHY_REG_ADDR_SHIFT);

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

int phy_write(int phy_addr, int phy_reg, u16 data)
{
	int err;
	u32 v, val = MII_BUSY;

	val |= (phy_addr << MII_PHY_ADDR_SHIFT);
	val |= (phy_reg << MII_PHY_REG_ADDR_SHIFT) | MII_WRITE;

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

static void phy_scan(void)
{
	int i;
	int phy_addr = PHY_ADDR;
	int val1, val2, phyid;

	for (i = 0; i < 2; i++) {
		val1 = phy_read(phy_addr, MII_PHYSID1);
		val2 = phy_read(phy_addr, MII_PHYSID2);
		phyid = (val1 << 16) | val2;
		dev_info(&pdev->dev, "%d: physid1=%08x physid2=%08x phyid=%08x\n",
			i, val1 & 0xFFFF, val2 & 0xFFFF, phyid);
	}
}

static int phy_reset(void)
{
	int ret;
	int phy_addr = PHY_ADDR;
	unsigned int retries = 12;

	ret = phy_write(phy_addr, MII_BMCR, BMCR_RESET);
	if (ret < 0) {
		dev_err(&pdev->dev, "PHY reset failed\n");
		return ret;
	}

	do {
		msleep(50);
		ret = phy_read(phy_addr, MII_BMCR);
		if (ret < 0)
			return ret;
	} while (ret & BMCR_RESET && --retries);
	if (ret & BMCR_RESET) {
		dev_err(&pdev->dev, "PHY reset timeout\n");
		return -ETIMEDOUT;
	}

	msleep(1);
	writel(0, emac_base_addr + MII_DATA);
	dev_info(&pdev->dev, "PHY reset done\n");
	return 0;
}

int phy_get_link_state(void)
{
	volatile int status;

	/* Do a fake read */
	status = phy_read(PHY_ADDR, MII_BMSR);
	if (status < 0)
		return status;

	status = phy_read(PHY_ADDR, MII_BMSR);
	if (status < 0)
		return status;

	dev_dbg(&pdev->dev, "BMSR=%08x\n", status);

	return (status & BMSR_LSTATUS) ? 1 : 0;
}

int phy_start(void)
{
	int ret;

	ret = phy_reset();
	if (ret)
		return ret;

	phy_scan();

	return 0;
}

int phy_stop(void)
{
	return 0;
}
