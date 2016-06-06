TEMPLATE = app
TARGET = lin_guider
QT += core \
    gui
CONFIG(debug, debug|release):DSTDIR = debug
else:DSTDIR = release
OBJECTS_DIR = $$DSTDIR/.obj
MOC_DIR = $$DSTDIR/.moc
UI_DIR = $$DSTDIR/.ui
RCC_DIR = $$DSTDIR/.rcc
HEADERS += include/drift_graph.h \
    include/target_graph.h \
    include/math/gmath_donuts.h \
    include/math/donuts_guide.h \
    include/math/guider_math.h \
    include/io_dev/io_nexstar.h \
    include/io_dev/io_skywatcher.h \
    include/io_dev/io_qhy5ii.h \
    include/video_dev/qhy5ii_core.h \
    include/video_dev/video_qhy5ii.h \
    include/io_dev/io_gpio.h \
    include/io_dev/io_gpusb.h \
    include/settings.h \
    include/filters.h \
    include/io_dev/io_qhy6.h \
    include/video_dev/qhy6_core.h \
    include/video_dev/video_qhy6.h \
    include/io_dev/io_null.h \
    include/io_dev/io_qhy5.h \
    include/io_dev/io_atik.h \
    include/io_dev/io_asi.h \
    include/io_dev/io_sx.h \
    include/io_dev/io_ftdi.h \
    include/io_dev/io_lpt.h \
    include/video_dev/video_null.h \
    include/video_dev/video_dsi2pro.h \
    include/video_dev/video_qhy5.h \
    include/video_dev/video_atik.h \
    include/video_dev/atik_core.h \
    include/video_dev/atikccdusb.h \
    include/video_dev/video_sx.h \
    include/video_dev/sx_core.h \
    include/video_dev/video_asi.h \
    include/video_dev/asi_core.h \
    include/video_dev/sxccdusb.h \
    include/video_dev/video_pwc.h \
    include/video_dev/video_uvc.h \
    include/bayer.h \
    include/lusb.h \
    include/video_dev/qhy5_core.h \
    include/server.h \
    include/common.h \
    include/mrecorder.h \
    include/avilib.h \
    include/about.h \
    include/fio.h \
    include/maindef.h \
    include/pwc-ioctl.h \
    include/params.h \
    include/setup_video.h \
    include/rcalibration.h \
    include/matr.h \
    include/vect.h \
    include/gmath.h \
    include/guider.h \
    include/scroll_graph.h \
    include/decoder.h \
    include/video.h \
    include/setup_driver.h \
    include/timer.h \
    include/utils.h \
    include/io_driver.h \
    include/lin_guider.h
SOURCES += src/drift_graph.cpp \
    src/target_graph.cpp \
    src/math/gmath_donuts.cpp \
    src/math/donuts_guide.cpp \
    src/math/guider_math.cpp \
    src/io_dev/io_nexstar.cpp \
    src/io_dev/io_skywatcher.cpp \
    src/io_dev/io_qhy5ii.cpp \
    src/io_dev/io_atik.cpp \
    src/io_dev/io_sx.cpp \
    src/video_dev/qhy5ii_core.cpp \
    src/video_dev/video_qhy5ii.cpp \
    src/video_dev/video_atik.cpp \
    src/io_dev/io_asi.cpp \
    src/video_dev/atik_core.cpp \
    src/video_dev/sxccdusb.cpp \
    src/video_dev/video_sx.cpp \
    src/video_dev/sx_core.cpp \
    src/video_dev/video_asi.cpp \
    src/video_dev/asi_core.cpp \
    src/io_dev/io_gpio.cpp \
    src/io_dev/io_gpusb.cpp \
    src/settings.cpp \
    src/filters.cpp \
    src/io_dev/io_qhy6.cpp \
    src/video_dev/qhy6_core.cpp \
    src/video_dev/video_qhy6.cpp \
    src/io_dev/io_null.cpp \
    src/io_dev/io_qhy5.cpp \
    src/io_dev/io_ftdi.cpp \
    src/io_dev/io_lpt.cpp \
    src/video_dev/video_null.cpp \
    src/video_dev/video_dsi2pro.cpp \
    src/video_dev/video_qhy5.cpp \
    src/video_dev/video_pwc.cpp \
    src/video_dev/video_uvc.cpp \
    src/bayer.cpp \
    src/lusb.cpp \
    src/video_dev/qhy5_core.cpp \
    src/server.cpp \
    src/common.cpp \
    src/avilib.cpp \
    src/mrecorder.cpp \
    src/about.cpp \
    src/fio.cpp \
    src/maindef.cpp \
    src/params.cpp \
    src/setup_video.cpp \
    src/rcalibration.cpp \
    src/matr.cpp \
    src/vect.cpp \
    src/gmath.cpp \
    src/guider.cpp \
    src/scroll_graph.cpp \
    src/decoder.cpp \
    src/video.cpp \
    src/setup_driver.cpp \
    src/main.cpp \
    src/utils.cpp \
    src/io_driver.cpp \
    src/lin_guider.cpp
FORMS += ui/settings.ui \
    ui/mrecorder.ui \
    ui/about.ui \
    ui/setup_video.ui \
    ui/rcalibration.ui \
    ui/guider.ui \
    ui/setup_driver.ui \
    ui/lin_guider.ui
RESOURCES += rc/lin_guider.qrc
INCLUDEPATH += include/ \
    include/io_dev/ \
    include/video_dev/ \
    include/math/ \
    ./
LIBS += -lusb-1.0 \
    -ldl \
    -lrt
