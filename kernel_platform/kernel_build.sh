#!/bin/sh
#======================================================================
#======================================================================
set -x

KERNEL_BUILD_CMD=""

KERNEL_BUILD_CMD+="TARGET_BUILD_VARIANT=userdebug BUILD_CONFIG=build.config VARIANT=consolidate ./build/build.sh -j72;"

/bin/bash -c "(${KERNEL_BUILD_CMD})" 2>&1 |tee compile_kernel_platform.log

