zephyr_library()
zephyr_library_sources_ifdef(CONFIG_RTC_RV3032 rtc_rv3032.c)

# Include the in-tree RTC drivers directory to access rtc_utils.h
zephyr_library_include_directories($ENV{ZEPHYR_BASE}/drivers/rtc)
