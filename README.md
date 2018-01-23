# Linux Kernel 3.10.107, EAE Touch Driver, Odroid-C0/C1

## Compilation Steps 
- install toolchain
- build kernel
- install kernel

### Install Toolchain
- Download toolchain click the link

http://releases.linaro.org/archive/14.09/components/toolchain/binaries/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux.tar.xz

```sh
$ sudo mkdir -p /opt/toolchains
$ sudo tar xvf gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux.tar.xz -C /opt/toolchains/
$ export ARCH=arm
$ export CROSS_COMPILE=arm-linux-gnueabihf-
$ export PATH=/opt/toolchains/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/:$PATH
$ source ~/.bashrc
```
### Build Kernel

```sh
$ make eae_defconfig
$ make <-j4>
$ make <-j4> modules
$ make uImage
```

### Install Kernel

- Plug the Boot-Device(eMMC or SD) into the USB memory CARD reader and Connect the USB memory CARD reader to your HOST PC(Linux OS).
- Copy the uImage and DT((meson8b_odroidc.dtb) to the FAT partition(1st partition) in the Boot-Device.

```sh
$ mkdir -p mount
$ sudo mount /dev/mmcblk0p1 ./mount
$ sudo cp arch/arm/boot/uImage arch/arm/boot/dts/meson8b_odroidc.dtb ./mount && sync && sudo umount ./mount
```
- Copy the driver modules to the EXT4 partition(2nd partition) in the Boot-Device.

```sh
$ sudo mount /dev/mmcblk0p2 ./mount
$ sudo make modules_install ARCH=arm INSTALL_MOD_PATH=./mount && sync && sudo umount ./mount
$ rm -rf mount
```

#### Referance
    https://wiki.odroid.com/odroid-c1/software/building_kernel