#!/bash -e

if ! [ "$(dpkg -s libssl-dev)" ]; then
    echo "Host is missing libssl-dev, please install first!"
    exit 1
fi

# Note the kernel symlink!
YOCTO_SDK_KRNL_SRC_SYMLINK="sysroots/cortexa72-cortexa53-brainlab-linux/usr/src/kernel"
YOCTO_SDK_KRNL_SRC="sysroots/cortexa72-cortexa53-brainlab-linux/lib/modules/6.1.30-xilinx-v2023.2/build"

if [ ! -e "$MY_YOCTO_SDK_BASE/$YOCTO_SDK_KRNL_SRC_SYMLINK" ]; then
    echo "SDK does not contain kernel sources!"
    exit 1
fi

# Prepare SDK, see https://stackoverflow.com/questions/60923890/how-to-build-linux-kernel-module-using-yocto-sdk
echo "****************"
echo "Preparing SDK installation for Kernel dev!"
pushd $MY_YOCTO_SDK_BASE/$YOCTO_SDK_KRNL_SRC || exit 1
source "$MY_YOCTO_SDK_BASE/environment-setup-cortexa72-cortexa53-brainlab-linux"
make scripts && make prepare

popd
