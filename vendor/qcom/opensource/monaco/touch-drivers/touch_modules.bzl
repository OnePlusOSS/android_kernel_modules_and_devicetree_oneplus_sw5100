# Importing to touch module entry api from touch_modules_build.bzl to define module entried for touch drivers
load(":touch_modules_build.bzl", "touch_module_entry")

# Importing the touch driver headers defined in BUILD.bazel
touch_driver_modules = touch_module_entry([":touch_drivers_headers"])

#Including the headers in the modules to be declared
module_entry = touch_driver_modules.register

#--------------- TOUCH-DRIVERS MODULES ------------------

#define ddk_module() for goodix_ts
module_entry(
    name = "goodix_ts",
    config_option = "CONFIG_TOUCHSCREEN_GOODIX_BRL",
    srcs = [
            "goodix_berlin_driver/goodix_brl_fwupdate.c",
            "goodix_berlin_driver/goodix_brl_hw.c",
            "goodix_berlin_driver/goodix_brl_i2c.c",
            "goodix_berlin_driver/goodix_brl_spi.c",
            "goodix_berlin_driver/goodix_cfg_bin.c",
            "goodix_berlin_driver/goodix_ts_core.c",
            "goodix_berlin_driver/goodix_ts_gesture.c",
            "goodix_berlin_driver/goodix_ts_inspect.c",
            "goodix_berlin_driver/goodix_ts_tools.c",
            "goodix_berlin_driver/goodix_ts_utils.c",
            "qts/qts_core.c"
    ]
)

#define ddk_module() for nt36xxx
module_entry(
    name = "nt36xxx-i2c",
    config_option = "CONFIG_TOUCHSCREEN_NT36XXX_I2C",
    srcs = [
            "nt36xxx/nt36xxx_ext_proc.c",
            "nt36xxx/nt36xxx_fw_update.c",
            "nt36xxx/nt36xxx_mp_ctrlram.c",
            "nt36xxx/nt36xxx.c"
    ]
)

#define ddk_module() for atmel_mxt_ts
module_entry(
    name = "atmel_mxt_ts",
    config_option = "CONFIG_TOUCHSCREEN_ATMEL_MXT",
    srcs = [
            "atmel_mxt/atmel_mxt_ts.c",
    ]
)

#define ddk_module() for dummy_ts
module_entry(
    name = "dummy_ts",
    config_option = "CONFIG_TOUCHSCREEN_DUMMY",
    srcs = [
            "dummy_touch/dummy_touch.c"
    ]
)