#!/bin/bash
# Version 0.3

## EXPORT ##
export ARCH=arm64
#export SUBARCH=arm
#export KBUILD_BUILD_USER="countstarlight"
#export KBUILD_BUILD_HOST="countstarlight"
export CROSS_COMPILE=/home/countstarlight/tools/prebuilts/linux-x86/aarch64/aarch64-linux-android-4.9/bin/aarch64-linux-android-

## MAKE ##
cd kernel-3.10
make r7plust_defconfig
make -j16
cd ..
mv kernel-3.10/arch/arm64/boot/Image.gz-dtb crlv/boot/boot.img-kernel

## REPACK ##
cd crlv
sh makebootimage.sh
cd ..
mv crlv/output/boot.img forpack/boot.img

## ZIPPING ##
cd forpack
zip  -r ../boot.zip ./
