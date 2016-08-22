## EXPORT ##
export ARCH=arm64
#export SUBARCH=arm
export KBUILD_BUILD_USER="countstarlight"
export KBUILD_BUILD_HOST="countstarlight"
export CROSS_COMPILE=/home/countstarlight/tools/prebuilts/linux-x86/aarch64/aarch64-linux-android-4.9/bin/aarch64-linux-android-

cd kernel-3.10
make mrproper
cd ..
rm forpack/boot.img
rm crlv/boot/boot.img-kernel
rm boot.zip
