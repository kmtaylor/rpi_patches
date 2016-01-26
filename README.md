# dm9601-bug.patch
Instruction how to patch and build the kernel:

```
sudo rpi-update
sudo apt-get -y update
sudo apt-get -y install gcc make bc screen ncurses-dev
git clone --depth 1 git://github.com/raspberrypi/linux.git
git clone https://github.com/kmtaylor/rpi_patches.git
cd linux
patch -p1 < ../rpi_patches/dm9601-bug.patch
KERNEL_SRC=/home/pi/linux/
make mrproper
sudo modprobe configs
zcat /proc/config.gz > .config
make oldconfig
make
sudo make modules_install
sudo cp arch/arm/boot/zImage /boot/kernel.img
```
