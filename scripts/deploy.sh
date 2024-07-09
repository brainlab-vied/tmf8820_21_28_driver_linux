#!/bin/bash -e

script=$(basename "$0")
if [ "$#" -ne 2 ]; then
    if [[ -z "${BV_IP}" ]]; then
        echo "BV_IP not defined. Usage $script <ip> <yocto sdk base>"
        exit 1
    else
        MY_BV_IP="${BV_IP}"
    fi
    if [[ -z "${YOCTO_SDK_BASE}" ]]; then
        echo "YOCTO_SDK_BASE not defined. Usage $script <ip> <yocto sdk base>"
        exit 1
    else
        MY_YOCTO_SDK_BASE=$(realpath ${YOCTO_SDK_BASE})
    fi
else
    MY_BV_IP=$1
    MY_YOCTO_SDK_BASE=$(realpath ${2})
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

echo "Ping $MY_BV_IP"
ping -c 1 -W 1 $MY_BV_IP > /dev/null

source $DIR/common.sh

echo "Assuring project is build"
$DIR/build.sh $MY_YOCTO_SDK_BASE

pushd $DIR/../
source "$MY_YOCTO_SDK_BASE/environment-setup-cortexa72-cortexa53-brainlab-linux"
export KERNEL_SRC=$MY_YOCTO_SDK_BASE/$YOCTO_SDK_KRNL_SRC_SYMLINK

echo "Preparing installation folder"
mkdir -p deploy
make INSTALL_MOD_PATH="$(pwd)/deploy" modules_install

echo "Deploying kernel module to device $1"
ssh root@$MY_BV_IP "mount -o remount,rw /"
scp -r deploy/* root@$MY_BV_IP:/
ssh root@$MY_BV_IP "mount -o remount,ro /"

popd
