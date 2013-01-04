CFILES = rasp_sc1602.c
LINUX_SRC = /home/pi/linux-src

obj-m += sc1602.o
sc1602-objs := $(CFILES:.c=.o)

all:
	make -C $(LINUX_SRC) M=$(PWD) modules

clean:
	make -C $(LINUX_SRC) M=$(PWD) clean
