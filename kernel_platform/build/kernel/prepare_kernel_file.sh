#!/bin/bash

ANDROID_KP_OUT_DIR=$1
ANDROID_KERNEL_OUT=$2
ANDROID_KERNEL_HEADER=$3

if [ -d ${ANDROID_KERNEL_OUT} ];then
    rm ${ANDROID_KERNEL_OUT} -rf
fi

if [ -d ${ANDROID_KERNEL_HEADER} ];then
    rm ${ANDROID_KERNEL_HEADER} -rf
fi

mkdir -p  ${ANDROID_KERNEL_OUT} ${ANDROID_KERNEL_HEADER}

first_stage_kos=$(mktemp)
echo "enter prepare kernel sh"
if [ -e ${ANDROID_KP_OUT_DIR}/dist/modules.load ]; then
   /bin/cat ${ANDROID_KP_OUT_DIR}/dist/modules.load | \
   /usr/bin/xargs -L 1 basename | \
   /usr/bin/xargs -L 1 find ${ANDROID_KP_OUT_DIR}/dist/ -name > ${first_stage_kos}
else
   find ${ANDROID_KP_OUT_DIR}/dist/ -name \*.ko > ${first_stage_kos}
fi

rm -f ${ANDROID_KERNEL_OUT}/*.ko ${ANDROID_KERNEL_OUT}/modules.*
if [ -s "${first_stage_kos}" ]; then
  cp $(/bin/cat ${first_stage_kos}) ${ANDROID_KERNEL_OUT}/
else
  echo "  WARNING!! No first stage modules found"
fi

if [ -e ${ANDROID_KP_OUT_DIR}/dist/modules.blocklist ]; then
  cp ${ANDROID_KP_OUT_DIR}/dist/modules.blocklist ${ANDROID_KERNEL_OUT}/modules.blocklist
fi

if [ -e ${ANDROID_KP_OUT_DIR}/dist/modules.load ]; then
  cp ${ANDROID_KP_OUT_DIR}/dist/modules.load ${ANDROID_KERNEL_OUT}/modules.load
fi

system_dlkm_kos=$(mktemp)
if [ -s ${ANDROID_KP_OUT_DIR}/dist/system_dlkm.modules.load ]; then
  /bin/cat ${ANDROID_KP_OUT_DIR}/dist/system_dlkm.modules.load | \
  /usr/bin/xargs -L 1 basename | \
  /usr/bin/xargs -L 1 find ${ANDROID_KP_OUT_DIR}/dist/ -name > ${system_dlkm_kos}
else
  echo "  system_dlkm_kos.modules.load file is not found or is empty"
fi

rm -rf ${ANDROID_KERNEL_OUT}/system_dlkm/*
if [ -s "${system_dlkm_kos}" ]; then
  mkdir -p ${ANDROID_KERNEL_OUT}/system_dlkm/
  # Unzip the system_dlkm staging tar copied from kernel_platform to system_dlkm out directory
  tar -xf ${ANDROID_KP_OUT_DIR}/dist/system_dlkm_staging_archive.tar.gz \
-C ${ANDROID_KERNEL_OUT}/system_dlkm/
else
  echo "  WARNING!! No system_dlkm (second stage) modules found"
fi

rm -f ${ANDROID_KERNEL_OUT}/vendor_dlkm/*.ko ${ANDROID_KERNEL_OUT}/vendor_dlkm/modules.*
second_stage_kos=$(find ${ANDROID_KP_OUT_DIR}/dist/ -name \*.ko | \
grep -v -F -f ${first_stage_kos} -f ${system_dlkm_kos} || true)
if [ -n "${second_stage_kos}" ]; then
  mkdir -p ${ANDROID_KERNEL_OUT}/vendor_dlkm
  cp ${second_stage_kos} ${ANDROID_KERNEL_OUT}/vendor_dlkm
else
  echo "  WARNING!! No vendor_dlkm (second stage) modules found"
fi

if [ -e ${ANDROID_KP_OUT_DIR}/dist/vendor_dlkm.modules.blocklist ]; then
  cp ${ANDROID_KP_OUT_DIR}/dist/vendor_dlkm.modules.blocklist \
    ${ANDROID_KERNEL_OUT}/vendor_dlkm/modules.blocklist
fi

if [ -s ${ANDROID_KP_OUT_DIR}/dist/vendor_dlkm.modules.load ]; then
  cp ${ANDROID_KP_OUT_DIR}/dist/vendor_dlkm.modules.load \
    ${ANDROID_KERNEL_OUT}/vendor_dlkm/modules.load
fi

if [ -e ${ANDROID_KP_OUT_DIR}/dist/system_dlkm.modules.blocklist ]; then
  cp ${ANDROID_KP_OUT_DIR}/dist/system_dlkm.modules.blocklist \
    ${ANDROID_KERNEL_OUT}/vendor_dlkm/system_dlkm.modules.blocklist
fi

for file in Image vmlinux System.map .config Module.symvers kernel-uapi-headers.tar.gz ; do
 cp ${ANDROID_KP_OUT_DIR}/dist/${file} ${ANDROID_KERNEL_OUT}/
done

cp ${ANDROID_KP_OUT_DIR}/dist/*.dtb* ${ANDROID_KERNEL_OUT}/
cp -rf ${ANDROID_KP_OUT_DIR}/dist/dtbo.img ${ANDROID_KERNEL_OUT}/
cp -rf ${ANDROID_KP_OUT_DIR}/dist/Image ${ANDROID_KERNEL_OUT}/
mv ${ANDROID_KERNEL_OUT}/Image ${ANDROID_KERNEL_OUT}/kernel


EXT_KERNEL_MOD_PATH=../vendor/qcom/opensource/monaco
KERNEL_OUT=${ANDROID_KP_OUT_DIR}/msm-kernel

mkdir -p ${ANDROID_KERNEL_HEADER}/linux \
	${ANDROID_KERNEL_HEADER}/linux/nfsd \
	${ANDROID_KERNEL_HEADER}/linux/usb \
	${ANDROID_KERNEL_HEADER}/media \
	${ANDROID_KERNEL_HEADER}/display \
	${ANDROID_KERNEL_HEADER}/display/drm \
	${ANDROID_KERNEL_HEADER}/display/hdcp \
	${ANDROID_KERNEL_HEADER}/display/media \
	${ANDROID_KERNEL_HEADER}/misc \
	${ANDROID_KERNEL_HEADER}/sound

cp -rf ../vendor/nxp/opensource/monaco/driver/include/uapi/linux/nfc/nfcinfo.h ${ANDROID_KERNEL_HEADER}/nfcinfo.h

cp -rf ${EXT_KERNEL_MOD_PATH}/display-drivers/include/uapi/display/drm/msm_drm_pp.h ${ANDROID_KERNEL_HEADER}/display/drm/msm_drm_pp.h
cp -rf ${EXT_KERNEL_MOD_PATH}/display-drivers/include/uapi/display/drm/sde_drm.h ${ANDROID_KERNEL_HEADER}/display/drm/sde_drm.h
cp -rf ${EXT_KERNEL_MOD_PATH}/display-drivers/include/uapi/display/hdcp/msm_hdmi_hdcp_mgr.h ${ANDROID_KERNEL_HEADER}/display/hdcp/msm_hdmi_hdcp_mgr.h
cp -rf ${EXT_KERNEL_MOD_PATH}/display-drivers/include/uapi/display/media/mmm_color_fmt.h ${ANDROID_KERNEL_HEADER}/display/media/mmm_color_fmt.h
cp -rf ${EXT_KERNEL_MOD_PATH}/display-drivers/include/uapi/display/media/msm_sde_rotator.h ${ANDROID_KERNEL_HEADER}/display/media/msm_sde_rotator.h
cp -rf ${EXT_KERNEL_MOD_PATH}/audio-kernel/include/uapi/audio/linux/msm_audio.h ${ANDROID_KERNEL_HEADER}/linux/msm_audio.h
cp -rf ${EXT_KERNEL_MOD_PATH}/securemsm-kernel/linux/smcinvoke.h ${ANDROID_KERNEL_HEADER}/linux/smcinvoke.h
cp -rf ${EXT_KERNEL_MOD_PATH}/audio-kernel/include/uapi/audio/sound/audio_compressed_formats.h ${ANDROID_KERNEL_HEADER}/sound/audio_compressed_formats.h
cp -rf ${EXT_KERNEL_MOD_PATH}/audio-kernel/include/uapi/audio/sound/audio_effects.h ${ANDROID_KERNEL_HEADER}/sound/audio_effects.h
cp -rf ${EXT_KERNEL_MOD_PATH}/audio-kernel/include/uapi/audio/sound/audio_slimslave.h ${ANDROID_KERNEL_HEADER}/sound/audio_slimslave.h
cp -rf ${EXT_KERNEL_MOD_PATH}/audio-kernel/include/uapi/audio/sound/devdep_params.h ${ANDROID_KERNEL_HEADER}/sound/devdep_params.h
cp -rf ${EXT_KERNEL_MOD_PATH}/audio-kernel/include/uapi/audio/sound/lsm_params.h ${ANDROID_KERNEL_HEADER}/sound/lsm_params.h
cp -rf ${EXT_KERNEL_MOD_PATH}/audio-kernel/include/uapi/audio/sound/msmcal-hwdep.h ${ANDROID_KERNEL_HEADER}/sound/msmcal-hwdep.h
cp -rf ${EXT_KERNEL_MOD_PATH}/audio-kernel/include/uapi/audio/sound/voice_params.h ${ANDROID_KERNEL_HEADER}/sound/voice_params.h
cp -rf ${EXT_KERNEL_MOD_PATH}/audio-kernel/include/uapi/audio/sound/wcd-dsp-glink.h ${ANDROID_KERNEL_HEADER}/sound/wcd-dsp-glink.h

cp -rf ${KERNEL_OUT}/usr/include/linux/esoc_ctrl.h ${ANDROID_KERNEL_HEADER}/linux/esoc_ctrl.h
cp -rf ${KERNEL_OUT}/usr/include/linux/gh_virtio_backend.h ${ANDROID_KERNEL_HEADER}/linux/gh_virtio_backend.h
cp -rf ${KERNEL_OUT}/usr/include/linux/gunyah.h ${ANDROID_KERNEL_HEADER}/linux/gunyah.h
cp -rf ${KERNEL_OUT}/usr/include/linux/ion.h ${ANDROID_KERNEL_HEADER}/linux/ion.h
cp -rf ${KERNEL_OUT}/usr/include/linux/ipa_qmi_service_v01.h ${ANDROID_KERNEL_HEADER}/linux/ipa_qmi_service_v01.h
cp -rf ${KERNEL_OUT}/usr/include/linux/mem-buf.h ${ANDROID_KERNEL_HEADER}/linux/mem-buf.h
cp -rf ${KERNEL_OUT}/usr/include/linux/mhi.h ${ANDROID_KERNEL_HEADER}/linux/mhi.h
cp -rf ${KERNEL_OUT}/usr/include/linux/msm_geni_serial.h ${ANDROID_KERNEL_HEADER}/linux/msm_geni_serial.h
cp -rf ${KERNEL_OUT}/usr/include/linux/msm_ion.h ${ANDROID_KERNEL_HEADER}/linux/msm_ion.h
cp -rf ${KERNEL_OUT}/usr/include/linux/msm_ion_ids.h ${ANDROID_KERNEL_HEADER}/linux/msm_ion_ids.h
cp -rf ${KERNEL_OUT}/usr/include/linux/msm_ipa.h ${ANDROID_KERNEL_HEADER}/linux/msm_ipa.h
cp -rf ${KERNEL_OUT}/usr/include/linux/msm_rmnet.h ${ANDROID_KERNEL_HEADER}/linux/msm_rmnet.h
cp -rf ${KERNEL_OUT}/usr/include/linux/msm_sysstats.h ${ANDROID_KERNEL_HEADER}/linux/msm_sysstats.h
cp -rf ${KERNEL_OUT}/usr/include/linux/qbt_handler.h ${ANDROID_KERNEL_HEADER}/linux/qbt_handler.h
cp -rf ${KERNEL_OUT}/usr/include/linux/qg.h ${ANDROID_KERNEL_HEADER}/linux/qg.h
cp -rf ${KERNEL_OUT}/usr/include/linux/qg-profile.h ${ANDROID_KERNEL_HEADER}/linux/qg-profile.h
cp -rf ${KERNEL_OUT}/usr/include/linux/qseecom.h ${ANDROID_KERNEL_HEADER}/linux/qseecom.h
cp -rf ${KERNEL_OUT}/usr/include/linux/qti_virtio_mem.h ${ANDROID_KERNEL_HEADER}/linux/qti_virtio_mem.h
cp -rf ${KERNEL_OUT}/usr/include/linux/rmnet_ipa_fd_ioctl.h ${ANDROID_KERNEL_HEADER}/linux/rmnet_ipa_fd_ioctl.h
cp -rf ${KERNEL_OUT}/usr/include/linux/slatecom_interface.h ${ANDROID_KERNEL_HEADER}/linux/slatecom_interface.h
cp -rf ${KERNEL_OUT}/usr/include/linux/spcom.h ${ANDROID_KERNEL_HEADER}/linux/spcom.h
cp -rf ${KERNEL_OUT}/usr/include/linux/spss_utils.h ${ANDROID_KERNEL_HEADER}/linux/spss_utils.h
cp -rf ${KERNEL_OUT}/usr/include/linux/nfsd/nfsfh.h ${ANDROID_KERNEL_HEADER}/linux/nfsd/nfsfh.h
cp -rf ${KERNEL_OUT}/usr/include/linux/usb/usb_ctrl_qti.h ${ANDROID_KERNEL_HEADER}/linux/usb/usb_ctrl_qti.h
cp -rf ${KERNEL_OUT}/usr/include/media/synx.h ${ANDROID_KERNEL_HEADER}/media/synx.h
cp -rf ${KERNEL_OUT}/usr/include/misc/adsp_sleepmon.h ${ANDROID_KERNEL_HEADER}/misc/adsp_sleepmon.h
