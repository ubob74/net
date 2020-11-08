net-y += emac.o phy.o emac_hw.o

obj-m += net.o

all:
	make -C $(KSRC) M=$(PWD) modules
clean:
	make -C $(KSRC) M=$(PWD) clean
	rm -f *.o *~

