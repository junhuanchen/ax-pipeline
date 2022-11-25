file(GLOB SRC_LIST "*.c" "pipe/*.c" "lvgl_sample/*.c" "../utilities/*.cpp")
file(GLOB SRC_LIST_COMMON "${BSP_MSP_DIR}/../sample/common/*.c" "${BSP_MSP_DIR}/../sample/common/common_codec/*.c" "${BSP_MSP_DIR}/../sample/common/vo/*.c")
file(GLOB SRC_LIST_SAMPLE_RUN_JOINT "../sample_run_joint/*.cpp")

aux_source_directory("lvgl/lvgl/src/lv_core"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lvgl/src/lv_draw"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lvgl/src/lv_font"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lvgl/src/lv_hal"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lvgl/src/lv_misc"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lvgl/src/lv_objx"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lvgl/src/lv_widgets"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lvgl/src/lv_themes"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lv_examples"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lv_examples/src/assets/"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lv_examples/src/lv_demo_widgets/"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lv_examples/src/lv_demo_benchmark/"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lv_examples/src/lv_ex_get_started/"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lv_examples/src/lv_ex_style/"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lv_examples/src/lv_ex_widgets/lv_ex_img/"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lv_examples/src/lv_ex_widgets/lv_ex_canvas/"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lv_lib_gif"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lv_lib_png"    SRC_LIST_LVGL)
aux_source_directory("lvgl/lv_lib_qrcode"   SRC_LIST_LVGL)
aux_source_directory("lvgl/lv_drivers/display"   SRC_LIST_LVGL)
aux_source_directory("lvgl/src"   SRC_LIST_LVGL)

add_executable(sample_vin_ivps_joint_vo_lvgl 
                ${SRC_LIST}
                ${SRC_LIST_COMMON}
                ${SRC_LIST_SAMPLE_RUN_JOINT}
                ${SRC_LIST_LVGL}
                )

# opencv
set (OpenCV_DIR ${CMAKE_SOURCE_DIR}/3rdparty/opencv-arm-linux/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
target_include_directories (sample_vin_ivps_joint_vo_lvgl PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(sample_vin_ivps_joint_vo_lvgl PRIVATE ${OpenCV_LIBS})


# drm
target_link_directories (sample_vin_ivps_joint_vo_lvgl PRIVATE ${BSP_MSP_DIR}/../../third-party/drm/lib)
target_link_libraries (sample_vin_ivps_joint_vo_lvgl PRIVATE drm)

target_include_directories (sample_vin_ivps_joint_vo_lvgl PRIVATE ${BSP_MSP_DIR}/../sample/common)
target_include_directories (sample_vin_ivps_joint_vo_lvgl PRIVATE ${BSP_MSP_DIR}/include)
target_include_directories (sample_vin_ivps_joint_vo_lvgl PRIVATE ${BSP_MSP_DIR}/include/npu_cv_kit)
target_link_directories (sample_vin_ivps_joint_vo_lvgl PRIVATE ${BSP_MSP_DIR}/lib)

target_link_libraries (sample_vin_ivps_joint_vo_lvgl PRIVATE pthread dl) # ax620a use this
# target_link_libraries (sample_vin_ivps_joint_vo PRIVATE pthread dl stdc++fs) # ax620u use this
target_link_libraries (sample_vin_ivps_joint_vo_lvgl PRIVATE ax_run_joint ax_interpreter_external ax_interpreter ax_sys axsyslog stdc++fs)
target_link_libraries (sample_vin_ivps_joint_vo_lvgl PRIVATE ax_venc ax_ivps ax_npu_cv_kit ax_3a ax_proton ax_mipi gomp ax_vo)
target_link_libraries (sample_vin_ivps_joint_vo_lvgl PRIVATE sns_os04a10 sns_os04a10_master sns_os04a10_slave sns_gc4653)

# lvgl
target_include_directories (sample_vin_ivps_joint_vo_lvgl PRIVATE lvgl)
target_include_directories (sample_vin_ivps_joint_vo_lvgl PRIVATE lvgl/include)
target_include_directories (sample_vin_ivps_joint_vo_lvgl PRIVATE lvgl_sample)


install (TARGETS sample_vin_ivps_joint_vo_lvgl DESTINATION bin)