net-test-y += net_test.o phy.o emac_hw.o

obj-m += net-test.o

all:
	make -C $(KSRC) M=$(PWD) modules
clean:
	make -C $(KSRC) M=$(PWD) clean
	rm -f *.o *~

