#!/bin/bash
#set -e
set -x

export PATH=/pkg/qct/software/python/3.8.2/bin:$PATH

ROOT_DIR=$(readlink -f $(dirname $0)/../..)
ANDROID_BUILD_TOP=${ROOT_DIR}/../android
ANDROID_PRODUCT_OUT=${ANDROID_BUILD_TOP}/out/target/product/monaco
if [ -z "${UNPACK_BOOTIMG_TOOL}" ]; then
UNPACK_BOOTIMG_TOOL="${ROOT_DIR}/tools/mkbootimg/unpack_bootimg.py"
fi
export OPLUS_UNPACK_IMG_BIN=${ROOT_DIR}/oplus_unpacking/img
export OPLUS_UNPACK_RAMDISK_BIN=${ROOT_DIR}/oplus_unpacking/ramdisk
rm -rf ${OPLUS_UNPACK_IMG_BIN}
rm -rf ${OPLUS_UNPACK_RAMDISK_BIN}
mkdir -p ${OPLUS_UNPACK_IMG_BIN}
if [ -e "${ANDROID_PRODUCT_OUT}/vendor_boot.img" ]; then
    echo "${ANDROID_PRODUCT_OUT}/vendor_boot.img ${OPLUS_UNPACK_IMG_BIN}"
    cp -f ${ANDROID_PRODUCT_OUT}/vendor_boot.img ${OPLUS_UNPACK_IMG_BIN}/
else
    echo "${ANDROID_PRODUCT_OUT}/vendor_boot.img not exist"
    exit 1
fi

OPLUS_UNPACK_RAMDISK_STRING=$(python3 "$UNPACK_BOOTIMG_TOOL" --boot_img="${OPLUS_UNPACK_IMG_BIN}/vendor_boot.img" \
    --out="${OPLUS_UNPACK_RAMDISK_BIN}/vendor_ramdisk/")

pushd ${OPLUS_UNPACK_RAMDISK_BIN}/vendor_ramdisk/
mv vendor_ramdisk vendor_ramdisk.gz
gunzip vendor_ramdisk.gz
file vendor_ramdisk
popd
#mkdir -p ${OPLUS_UNPACK_RAMDISK_BIN}/vendor_ramdisk/out
#pushd ${OPLUS_UNPACK_RAMDISK_BIN}/vendor_ramdisk/out
#cpio -ivdu < ${OPLUS_UNPACK_RAMDISK_BIN}/vendor_ramdisk/vendor_ramdisk
#pwd
#ls -l
#popd

OPLUS_VENDOR_KERNEL_CMDLINE=$(echo ${OPLUS_UNPACK_RAMDISK_STRING} | sed -n 's/.*vendor command line args: \(.*\) kernel tags load.*/\1/p')
#OPLUS_VENDOR_KERNEL_CMDLINE+=" buildvariant.buildvariant=userdebug"
OPLUS_VENDOR_KERNEL_BOOTCONFIG=$(cat ${OPLUS_UNPACK_RAMDISK_BIN}/vendor_ramdisk/bootconfig | tr "\n" " ")
OPLUS_VENDOR_RAMDISK_BINARY=${OPLUS_UNPACK_RAMDISK_BIN}/vendor_ramdisk/vendor_ramdisk
# OPLUS_BUILD_CMD=("BUILD_CONFIG=build.config")
# OPLUS_BUILD_CMD+=("SKIP_IF_VERSION_MATCHES=true")
# OPLUS_BUILD_CMD+=("FAST_BUILD=true")
# OPLUS_BUILD_CMD+=("VENDOR_RAMDISK_BINARY=${OPLUS_VENDOR_RAMDISK_BINARY}")
# OPLUS_BUILD_CMD+=("VENDOR_BOOTCONFIG=\"${OPLUS_VENDOR_KERNEL_BOOTCONFIG}\"")
# OPLUS_BUILD_CMD+=("KERNEL_VENDOR_CMDLINE=\"${OPLUS_VENDOR_KERNEL_CMDLINE}\"")

cd ${ROOT_DIR}
#env -i bash -c '"${OPLUS_BUILD_CMD[@]}" ./build/build.sh -j72 2>&1 | tee compile_kernel_platform.log'
env -i bash -c "BUILD_CONFIG=build.config VARIANT=gki SKIP_IF_VERSION_MATCHES=true FAST_BUILD=true VENDOR_RAMDISK_BINARY=${OPLUS_VENDOR_RAMDISK_BINARY} VENDOR_BOOTCONFIG=\"${OPLUS_VENDOR_KERNEL_BOOTCONFIG}\" KERNEL_VENDOR_CMDLINE=\"${OPLUS_VENDOR_KERNEL_CMDLINE}\" ./build/build.sh -j72 2>&1 | tee compile_kernel_platform.log"
echo $(date +"%T")
tail -n 10 compile_kernel_platform.log | grep 'Created vendor_boot.img'
if [[ $? -ne 0 ]]; then
    echo "build fail"
else
    echo "build success"
    env -i bash -c "./build/kernel/prepare_kernel_file.sh ./out/msm-kernel-monaco-gki/ ${ANDROID_BUILD_TOP}/device/qcom/monaco-kernel/"
fi

set +x

