#!/bin/bash -e

script=$(basename "$0")
if [ "$#" -ne 1 ]; then
    if [[ -z "${YOCTO_SDK_BASE}" ]]; then
        echo "YOCTO_SDK_BASE not defined. Usage $script <yocto sdk base>"
        exit 1
    else
        MY_YOCTO_SDK_BASE=$(realpath ${YOCTO_SDK_BASE})
    fi
else
    MY_YOCTO_SDK_BASE=$(realpath ${1})
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source $DIR/common.sh

pushd $DIR/../
source "$MY_YOCTO_SDK_BASE/environment-setup-cortexa72-cortexa53-brainlab-linux"
export KERNEL_SRC=$MY_YOCTO_SDK_BASE/$YOCTO_SDK_KRNL_SRC_SYMLINK

make clean
rm -rf deploy

popd
