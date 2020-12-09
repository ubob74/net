#ifndef _PHY_H_
#define _PHY_H_

#define HALF_DUPLEX			0
#define FULL_DUPLEX			1

#define SPEED10				2
#define SPEED100			3

struct device;

#define PHY_ADDR			1

int phy_setup(void);
void phy_release(void);
int phy_read(int phy_addr, int phy_reg);
int phy_write(int phy_addr, int phy_reg, u16 data);
int phy_start(void);
int phy_stop(void);
int phy_adjust_link(int *, int *);

#endif
