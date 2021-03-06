#-------------------------------------------------
#
# Project created by QtCreator 2018-05-18T21:04:20
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = myplottest
TEMPLATE = app
CONFIG += c++14
INCLUDEPATH+=\
    /home/skloe/Coding/CPP1X/USV/DPfloatover/third_party/eigen   \
    /opt/mosek/7/tools/platform/linux32x86/h \
    /home/skloe/Coding/CPP1X/USV/DPfloatover/include \
    /home/skloe/Coding/CPP1X/USV/timer \
    ../third_party/Profinet/src/source/pnd/src/agent \
    ../third_party/Profinet/src/source/pnd/src/cfg \
    ../third_party/Profinet/src/source/pnd/src/common\
    ../third_party/Profinet/src/source/pnd/src/io_base_core \
    ../third_party/Profinet/src/source/pnd/src/params \
    ../third_party/Profinet/src/source/pnd/src/params/config_xml/ \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/cfg/ \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/h/ \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/src/ \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core \
    ../third_party/Profinet/src/source/pnd/rmos \
    ../third_party/Profinet/src/source/pnd/rmos \
    ../third_party/Profinet/src/source/pnd/WpdPack/Include/  \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/  \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/mem3/cfg/  \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/lsa/  \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/cfg/  \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/timer/cfg/  \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/psi/  \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/pncore/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/lsa/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/pnio/  \
    ../third_party/Profinet/src/source/pnio/src/common/  \
    ../third_party/Profinet/src/source/edds/src/common/  \
    ../third_party/Profinet/src/source/edds/src/intel/  \
    ../third_party/Profinet/src/source/edds/src/inc/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/edds/  \
    ../third_party/Profinet/src/source/pnio/src/inc/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/acp/  \
    ../third_party/Profinet/src/source/acp/src/common/  \
    ../third_party/Profinet/src/source/acp/src/inc/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/cm/  \
    ../third_party/Profinet/src/source/cm/src/common/  \
    ../third_party/Profinet/src/source/cm/src/inc/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/clrpc/  \
    ../third_party/Profinet/src/source/clrpc/src/common/  \
    ../third_party/Profinet/src/source/clrpc/src/inc/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/dcp/  \
    ../third_party/Profinet/src/source/dcp/src/common/  \
    ../third_party/Profinet/src/source/dcp/src/inc/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/hif/  \
    ../third_party/Profinet/src/source/hif/src/common/  \
    ../third_party/Profinet/src/source/hif/src/inc/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/lldp/  \
    ../third_party/Profinet/src/source/lldp/src/common/  \
    ../third_party/Profinet/src/source/lldp/src/inc/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/mrp/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/nare/  \
    ../third_party/Profinet/src/source/nare/src/common/  \
    ../third_party/Profinet/src/source/nare/src/inc/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/oha/  \
    ../third_party/Profinet/src/source/oha/src/common/  \
    ../third_party/Profinet/src/source/oha/src/inc/  \
    ../third_party/Profinet/src/source/psi/src/common/  \
    ../third_party/Profinet/src/source/psi/src/inc/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/sock/  \
    ../third_party/Profinet/src/source/sock/src/common/  \
    ../third_party/Profinet/src/source/sock/src/inc/  \
    ../third_party/Profinet/src/source/psi/src/pnstack/tcip/  \
    ../third_party/Profinet/src/source/tcip/src/common/  \
    ../third_party/Profinet/src/source/tcip/src/inc/  \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/allports/  \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/  \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/ip/  \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/snmp/  \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/snmpv1/  \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/ipmc/  \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/tcp/  \
    ../third_party/Profinet/src/source/mem3/inc/  \
    ../third_party/Profinet/src/source/mem3/common/inc/  \
    ../third_party/Profinet/src/source/mem3/createifc/inc/  \
    ../third_party/Profinet/src/source/mem3/doubledyn/inc/  \
    ../third_party/Profinet/src/source/mem3/fixraster/inc/  \
    ../third_party/Profinet/src/source/mem3/incl/  \
    ../third_party/Profinet/src/source/mem3/universal/inc/  \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/posix/  \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/pntrc/  \
    ../third_party/Profinet/src/source/pntrc/src/common/  \
    ../third_party/Profinet/src/source/pntrc/src/inc/  \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/  \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/src/PnDev_DriverU.dll  \
    ../third_party/Profinet/src/source/pnd/src/common/  \
    ../third_party/Profinet/src/source/pnd/src/inc/  \
    ../third_party/Profinet/src/source/pnd/src/cfg/  \
    ../third_party/Profinet/src/source/pnd/src/params/  \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/cfg/  \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/h/  \
    ../third_party/Profinet/src/source/pnd/src/params/config_ini/  \
    ../third_party/Profinet/src/source/pnd/src/params/config_xml/  \
    ../third_party/Profinet/src/source/pnd/src/io_base_core/  \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/  \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/base/  \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/ioc/  \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/iodu/  \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/oha/  \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/pd/  \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/pnstack/  \
    ../third_party/Profinet/src/examples/test_app/src/




QMAKE_LFLAGS += -m32 -pthread -O0 -g3 -Wall -fmessage-length=0 -MD -MP -Wno-write-strings -Wno-unused-but-set-variable -Wno-type-limits -Wno-sign-compare -Wno-address -Wno-format
QMAKE_CXXFLAGS +=  -c -m32 -pthread -g3 -Wall -fmessage-length=0 -MMD -Wno-write-strings -Wno-unused-but-set-variable -Wno-type-limits -Wno-sign-compare -Wno-address -Wno-format
                   -std=c++14 \
                  -mtune=corei7-avx -march=corei7-avx -O0 -mavx2

LIBS += -L/opt/mosek/7/tools/platform/linux32x86/bin \
        -Wl,-rpath-link,/opt/mosek/7/tools/platform/linux32x86/bin \
        -Wl,-rpath,'/opt/mosek/7/tools/platform/linux32x86/bin' \
        -lsqlite3 \
        -lmosek \
        -pthread \
        -lc     \
        -lm     \
        /home/skloe/Coding/CPP1X/USV/DPfloatover/third_party/Profinet/src/examples/shared/linux32/build/lib/libpndriver.a \
        -lpthread \
        -lrt \
        -ldl


# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warning
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS \
            PNDEV_OS_LINUX \
            PNIO_PNDRIVER \
            EDDS_CFG_HW_INTEL \
            EPS_PSI_CFG_PLATFORM_H=pnd_psi_cfg_plf_linux_intel_interniche.h \
            _CRT_SECURE_NO_WARNINGS \
            EPS_CFG_DO_NOT_USE_TGROUPS \
            TOOL_CHAIN_GNU \
            EPS_RTOS_CFG_INCLUDE_H=eps_posix_cfg_linux.h \
            PNTRC_CFG_CUSTOM_SUBSYSTEM_HEADER=pndriver_pntrc_sub.h \
            _SYS_STDLIB_H_ \
            NO_DEBUG\
            _GNU_SOURCE\
            PNDEV_TEST_INTERNAL \
            UINT_ALREADY \
            _SYS_SELECT_H \
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
    display2ddialog.cpp \
    mainwindow.cpp \
    qcustomplot.cpp \
    dialogsetparameter.cpp \
    dialogfixedsetpoint.cpp \
    dialogstraightline.cpp \
    dialogrotation.cpp \
    dialogcooperation.cpp \
    ../network/src/crccheck.c \
    ../network/src/datapack.c \
    ../network/libcrc/src/crc8.c \
    ../network/libcrc/src/crc16.c \
    ../network/libcrc/src/crc32.c \
    ../network/libcrc/src/crc64.c \
    ../network/libcrc/src/crcccitt.c \
    ../network/libcrc/src/crcdnp.c \
    ../network/libcrc/src/crckrmit.c \
    ../network/libcrc/src/crcsick.c \
    ../network/libcrc/src/nmea-chk.c \
    ../joystick/src/joystick.cpp \
    ../network/src/pnd_test_iob_core.cpp \
    ../motioncapture/src/qtmMarkup.cpp \
    ../motioncapture/src/qtmNetwork.cpp \
    ../motioncapture/src/qtmOperations.cpp \
    ../motioncapture/src/qtmOutput.cpp \
    ../motioncapture/src/RTPacket.cpp \
    ../motioncapture/src/RTProtocol.cpp \
    dialogabout.cpp \
    dialogthrusterdiagi.cpp \
    dialogthrusterdiagii.cpp \
    dialogthrusterdiagiii.cpp \
    dialog6dof.cpp \
    csvstream.cpp




HEADERS += \
    globalvar.h \
    mainwindow.h \
    qcustomplot.h \
    display2ddialog.h \
    dialogsetparameter.h \
    dialogfixedsetpoint.h \
    dialogstraightline.h \
    dialogrotation.h \
    dialogcooperation.h \
    ../include/constants.h \
    ../include/threaded_Loop.h \
    ../include/realtimedata.h \
    ../network/include/crccheck.h \
    ../network/include/datapack.h \
    ../network/include/tcpserver_t.h \
    ../network/include/pnserver_t.h  \
    ../network/libcrc/include/checksum.h \
    ../motioncapture/include/motioncapture.h \
    ../sql/include/databasecpp.h \
    ../joystick/include/gamepadmonitor.h \
    ../joystick/include/joystick.h \
    ../controller/pidcontroller/include/controller.h \
    ../controller/pidcontroller/include/kalmanfilter.h \
    ../controller/pidcontroller/include/pidcontroller.h \
    ../controller/pidcontroller/include/thrusterallocation.h \
    ../third_party/Profinet/src/source/acp/src/common/acp_low.h \
    ../third_party/Profinet/src/source/acp/src/common/acp_pls.h \
    ../third_party/Profinet/src/source/acp/src/common/acp_sys.h \
    ../third_party/Profinet/src/source/acp/src/common/acp_trc.h \
    ../third_party/Profinet/src/source/acp/src/common/acp_usr.h \
    ../third_party/Profinet/src/source/acp/src/inc/acp_int.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_lib.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_low.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_pls.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_sys.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_trc.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_usr.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_uuid.h \
    ../third_party/Profinet/src/source/clrpc/src/inc/clrpc_icl.h \
    ../third_party/Profinet/src/source/clrpc/src/inc/clrpc_int.h \
    ../third_party/Profinet/src/source/clrpc/src/inc/clrpc_isv.h \
    ../third_party/Profinet/src/source/clrpc/src/inc/clrpc_pdu.h \
    ../third_party/Profinet/src/source/cm/src/common/cm_arcb.h \
    ../third_party/Profinet/src/source/acp/src/common/acp_low.h \
    ../third_party/Profinet/src/source/acp/src/common/acp_pls.h \
    ../third_party/Profinet/src/source/acp/src/common/acp_sys.h \
    ../third_party/Profinet/src/source/acp/src/common/acp_trc.h \
    ../third_party/Profinet/src/source/acp/src/common/acp_usr.h \
    ../third_party/Profinet/src/source/acp/src/inc/acp_int.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_lib.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_low.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_pls.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_sys.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_trc.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_usr.h \
    ../third_party/Profinet/src/source/clrpc/src/common/clrpc_uuid.h \
    ../third_party/Profinet/src/source/clrpc/src/inc/clrpc_icl.h \
    ../third_party/Profinet/src/source/clrpc/src/inc/clrpc_int.h \
    ../third_party/Profinet/src/source/clrpc/src/inc/clrpc_isv.h \
    ../third_party/Profinet/src/source/clrpc/src/inc/clrpc_pdu.h \
    ../third_party/Profinet/src/source/cm/src/common/cm_arcb.h \
    ../third_party/Profinet/src/source/cm/src/common/cm_arcb_helper.h \
    ../third_party/Profinet/src/source/cm/src/common/cm_err.h \
    ../third_party/Profinet/src/source/cm/src/common/cm_lib.h \
    ../third_party/Profinet/src/source/cm/src/common/cm_list.h \
    ../third_party/Profinet/src/source/cm/src/common/cm_low.h \
    ../third_party/Profinet/src/source/cm/src/common/cm_pls.h \
    ../third_party/Profinet/src/source/cm/src/common/cm_sys.h \
    ../third_party/Profinet/src/source/cm/src/common/cm_trc.h \
    ../third_party/Profinet/src/source/cm/src/common/cm_usr.h \
    ../third_party/Profinet/src/source/cm/src/inc/cm_argr.h \
    ../third_party/Profinet/src/source/cm/src/inc/cm_icl.h \
    ../third_party/Profinet/src/source/cm/src/inc/cm_iclar.h \
    ../third_party/Profinet/src/source/cm/src/inc/cm_imc.h \
    ../third_party/Profinet/src/source/cm/src/inc/cm_int.h \
    ../third_party/Profinet/src/source/cm/src/inc/cm_ipd.h \
    ../third_party/Profinet/src/source/cm/src/inc/cm_isv.h \
    ../third_party/Profinet/src/source/cm/src/inc/cm_isvar.h \
    ../third_party/Profinet/src/source/cm/src/inc/cm_md5.h \
    ../third_party/Profinet/src/source/cm/src/inc/cm_pdu.h \
    ../third_party/Profinet/src/source/cm/src/inc/cm_prm.h \
    ../third_party/Profinet/src/source/cm/src/inc/cm_rpc.h \
    ../third_party/Profinet/src/source/dcp/src/common/dcp_low.h \
    ../third_party/Profinet/src/source/dcp/src/common/dcp_pls.h \
    ../third_party/Profinet/src/source/dcp/src/common/dcp_sys.h \
    ../third_party/Profinet/src/source/dcp/src/common/dcp_trc.h \
    ../third_party/Profinet/src/source/dcp/src/common/dcp_usr.h \
    ../third_party/Profinet/src/source/dcp/src/inc/dcp_int.h \
    ../third_party/Profinet/src/source/edds/src/common/edds_iobuf_out.h \
    ../third_party/Profinet/src/source/edds/src/common/edds_iobuf_usr.h \
    ../third_party/Profinet/src/source/edds/src/common/edds_out.h \
    ../third_party/Profinet/src/source/edds/src/common/edds_sys.h \
    ../third_party/Profinet/src/source/edds/src/common/edds_trc.h \
    ../third_party/Profinet/src/source/edds/src/common/edds_usr.h \
    ../third_party/Profinet/src/source/edds/src/inc/edds_dbg.h \
    ../third_party/Profinet/src/source/edds/src/inc/edds_dev.h \
    ../third_party/Profinet/src/source/edds/src/inc/edds_int.h \
    ../third_party/Profinet/src/source/edds/src/inc/edds_iobuf_int.h \
    ../third_party/Profinet/src/source/edds/src/inc/edds_llif.h \
    ../third_party/Profinet/src/source/edds/src/inc/edds_nrt_ext.h \
    ../third_party/Profinet/src/source/edds/src/inc/edds_nrt_inc.h \
    ../third_party/Profinet/src/source/edds/src/inc/edds_pck.h \
    ../third_party/Profinet/src/source/edds/src/inc/edds_srt_ext.h \
    ../third_party/Profinet/src/source/edds/src/inc/edds_srt_inc.h \
    ../third_party/Profinet/src/source/edds/src/inc/edds_toolbox.h \
    ../third_party/Profinet/src/source/edds/src/intel/intel_cfg.h \
    ../third_party/Profinet/src/source/edds/src/intel/intel_inc.h \
    ../third_party/Profinet/src/source/edds/src/intel/intel_reg.h \
    ../third_party/Profinet/src/source/edds/src/intel/intel_usr.h \
    ../third_party/Profinet/src/source/hif/src/common/hif_mem_ring.h \
    ../third_party/Profinet/src/source/hif/src/common/hif_pls.h \
    ../third_party/Profinet/src/source/hif/src/common/hif_sys.h \
    ../third_party/Profinet/src/source/hif/src/common/hif_trc.h \
    ../third_party/Profinet/src/source/hif/src/common/hif_usr.h \
    ../third_party/Profinet/src/source/hif/src/inc/hif_int.h \
    ../third_party/Profinet/src/source/lldp/src/common/lldp_low.h \
    ../third_party/Profinet/src/source/lldp/src/common/lldp_sys.h \
    ../third_party/Profinet/src/source/lldp/src/common/lldp_usr.h \
    ../third_party/Profinet/src/source/lldp/src/inc/lldp_int.h \
    ../third_party/Profinet/src/source/lldp/src/inc/lldp_trc.h \
    ../third_party/Profinet/src/source/mem3/inc/mem3defaultmac.h \
    ../third_party/Profinet/src/source/mem3/inc/mem3if.h \
    ../third_party/Profinet/src/source/nare/src/common/nare_low.h \
    ../third_party/Profinet/src/source/nare/src/common/nare_out.h \
    ../third_party/Profinet/src/source/nare/src/common/nare_sys.h \
    ../third_party/Profinet/src/source/nare/src/common/nare_trc.h \
    ../third_party/Profinet/src/source/nare/src/common/nare_usr.h \
    ../third_party/Profinet/src/source/nare/src/inc/nare_dbg.h \
    ../third_party/Profinet/src/source/nare/src/inc/nare_int.h \
    ../third_party/Profinet/src/source/oha/src/common/oha_lib.h \
    ../third_party/Profinet/src/source/oha/src/common/oha_low.h \
    ../third_party/Profinet/src/source/oha/src/common/oha_sys.h \
    ../third_party/Profinet/src/source/oha/src/common/oha_trc.h \
    ../third_party/Profinet/src/source/oha/src/common/oha_usr.h \
    ../third_party/Profinet/src/source/oha/src/inc/oha_agnt.h \
    ../third_party/Profinet/src/source/oha/src/inc/oha_asn1.h \
    ../third_party/Profinet/src/source/oha/src/inc/oha_db.h \
    ../third_party/Profinet/src/source/oha/src/inc/oha_int.h \
    ../third_party/Profinet/src/source/oha/src/inc/oha_mibs.h \
    ../third_party/Profinet/src/source/oha/src/inc/oha_snmp.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/pntrc/pntrc_cfg.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/pntrc/pntrc_inc.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/pntrc/pntrc_pck.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/posix/eps_posix_cfg_linux.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/posix/eps_posix_cfg_windows.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/psi/psi_cfg.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/etc/psi/psi_inc.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_app.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_cp_hw.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_cp_mem.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_events.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_hif_drv_if.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_hif_short_drv.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_hif_universal_drv.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_hw_edds.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_hw_ertec200.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_hw_ertec400.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_hw_pnip.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_hw_soc.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_ipc.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_isr.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_lib.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_locks.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_mem.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_noshmdrv.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_plf.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_plf_types.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_pn_drv_if.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_pn_imcea_drv.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_pndevdrv.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_register.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_rtos.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_shm_file.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_shm_if.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_shm_if_config.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_stdmacdrv.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_sys.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_tasks.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_timer.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_timer_trc.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_trc.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/eps_wpcapdrv.h \
    ../third_party/Profinet/src/source/pnboards/eps/epssys/src/ltrc_if.h \
    ../third_party/Profinet/src/source/pnd/src/cfg/pnd_psi_cfg_plf_linux_intel_interniche.h \
    ../third_party/Profinet/src/source/pnd/src/cfg/pnd_psi_cfg_plf_windows_wpcap_interniche.h \
    ../third_party/Profinet/src/source/pnd/src/common/pndriver_pntrc_sub.h \
    ../third_party/Profinet/src/source/pnd/src/common/pniobase.h \
    ../third_party/Profinet/src/source/pnd/src/common/pnioerrx.h \
    ../third_party/Profinet/src/source/pnd/src/common/pniousrx.h \
    ../third_party/Profinet/src/source/pnd/src/common/servusrx.h \
    ../third_party/Profinet/src/source/pnd/src/inc/pnd_int.h \
    ../third_party/Profinet/src/source/pnd/src/inc/pnd_sys.h \
    ../third_party/Profinet/src/source/pnd/src/inc/pnd_trc.h \
    ../third_party/Profinet/src/source/pnd/src/io_base_core/pnd_iob_core.h \
    ../third_party/Profinet/src/source/pnd/src/io_base_core/pnd_iodu.h \
    ../third_party/Profinet/src/source/pnd/src/io_base_core/pnd_iodu_edds.h \
    ../third_party/Profinet/src/source/pnd/src/params/config_xml/pnd_xml_cfg.h \
    ../third_party/Profinet/src/source/pnd/src/params/config_xml/pnd_xml_helper.h \
    ../third_party/Profinet/src/source/pnd/src/params/config_xml/pnd_xml_std_interface.h \
    ../third_party/Profinet/src/source/pnd/src/params/config_xml/pnd_XmlParamStore.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/cfg/PARSP_Config.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/cfg/PARSP_Config_default.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/cfg/PARSP_Properties.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/cfg/PARSP_Types.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/cfg/PARSP_Utils.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/cfg/PARSP_XMLTags.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/h/PARS_ConversionService.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/h/PARS_ErrorCodes.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/h/PARS_FileSystem.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/h/PARS_MCB.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/h/PARS_MemoryService.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/h/PARS_Stream.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/h/PARS_Types.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/h/PARS_Version.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/h/PARS_XML.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/h/PARS_XML_Internal.h \
    ../third_party/Profinet/src/source/pnd/src/params/xml_parser/h/PARSP_File.h \
    ../third_party/Profinet/src/source/pnd/src/params/pnd_ParamStore.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/base/pnd_BgzdStore.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/base/pnd_UserNode.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/base/pnd_UserStore.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/base/pnd_ValueHelper.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/ioc/pnd_IOCAr.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/ioc/pnd_IOCDevice.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/ioc/pnd_IOCEventHandler.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/ioc/pnd_IOCSlot.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/ioc/pnd_IOCSubslot.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/ioc/pnd_IOCUser.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/oha/pnd_OHAEventHandler.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/oha/pnd_OHAUser.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/pd/pnd_PDEventHandler.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/pd/pnd_PDSubslot.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/pd/pnd_PDUser.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/pnstack/pnd_pnstack.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/pnd_pnio_user_core.h \
    ../third_party/Profinet/src/source/pnd/src/pnio_user_core/pnd_PnioUserCoreApp.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/Adonis_Inc.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/os_Env.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/PnCore.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/PnCore_Inc.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/pndev_adapt.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/pndev_config.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/PnDev_Driver_Inc.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/PnDev_DriverU.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/pndev_np_common.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/pndev_sbl_s7p.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/pndev_targetver.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/PnDev_Util.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/common/precomp.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/src/PnDev_DriverU.dll/clsDataSet.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/src/PnDev_DriverU.dll/clsDmm.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/src/PnDev_DriverU.dll/clsFile.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/src/PnDev_DriverU.dll/clsString.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/src/PnDev_DriverU.dll/clsUsedDevice.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/src/PnDev_DriverU.dll/Inc.h \
    ../third_party/Profinet/src/source/pndevdrv/PnDev_Driver/src/PnDev_DriverU.dll/stdafx.h \
    ../third_party/Profinet/src/source/pnio/src/common/edd_usr.h \
    ../third_party/Profinet/src/source/pnio/src/common/irte_rev05_rsl.h \
    ../third_party/Profinet/src/source/pnio/src/common/irte_rev06_rsl.h \
    ../third_party/Profinet/src/source/pnio/src/common/irte_rev07_rsl.h \
    ../third_party/Profinet/src/source/pnio/src/common/irte_rsl.h \
    ../third_party/Profinet/src/source/pnio/src/common/mem3cfg.h \
    ../third_party/Profinet/src/source/pnio/src/common/pnip_xrsl.h \
    ../third_party/Profinet/src/source/pnio/src/common/pntrc_sub.h \
    ../third_party/Profinet/src/source/pnio/src/inc/pnio_version.h \
    ../third_party/Profinet/src/source/pntrc/src/common/pntrc_pls.h \
    ../third_party/Profinet/src/source/pntrc/src/common/pntrc_sys.h \
    ../third_party/Profinet/src/source/pntrc/src/common/pntrc_trc.h \
    ../third_party/Profinet/src/source/pntrc/src/common/pntrc_usr.h \
    ../third_party/Profinet/src/source/pntrc/src/inc/pntrc_int.h \
    ../third_party/Profinet/src/source/pntrc/src/inc/pntrc_tbb.h \
    ../third_party/Profinet/src/source/psi/src/common/psi_pls.h \
    ../third_party/Profinet/src/source/psi/src/common/psi_sys.h \
    ../third_party/Profinet/src/source/psi/src/common/psi_trc.h \
    ../third_party/Profinet/src/source/psi/src/common/psi_usr.h \
    ../third_party/Profinet/src/source/psi/src/inc/psi_hd.h \
    ../third_party/Profinet/src/source/psi/src/inc/psi_int.h \
    ../third_party/Profinet/src/source/psi/src/inc/psi_ld.h \
    ../third_party/Profinet/src/source/psi/src/inc/psi_pnstack.h \
    ../third_party/Profinet/src/source/psi/src/inc/psi_res_calc.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/acp/acp_cfg.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/acp/acp_inc.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/acp/acp_pck.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/clrpc/clrpc_cfg.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/clrpc/clrpc_inc.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/clrpc/clrpc_pck.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/clrpc/clrpc_unpck.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/cm/cm_cfg.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/cm/cm_inc.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/dcp/dcp_cfg.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/dcp/dcp_inc.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/edds/edds_cfg.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/edds/edds_inc.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/edds/edds_iobuf_inc.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/edds/edds_pck.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/hif/hif_cfg.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/hif/hif_inc.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/lldp/lldp_cfg.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/lldp/lldp_inc.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/lsa/lsa_cfg.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/lsa/lsa_sys.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/lsa/lsa_usr.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/nare/nare_cfg.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/nare/nare_inc.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/oha/oha_cfg.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/oha/oha_inc.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/oha/oha_pck.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/pnio/edd_cfg.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/pnio/edd_inc.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/pnio/pnio_pck1_on.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/pnio/pnio_pck2_on.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/pnio/pnio_pck4_on.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/pnio/pnio_pck_off.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/sock/sock_cfg.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/sock/sock_inc.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/tcip/sy_off_pack_2.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/tcip/sy_on_pack_2.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/tcip/tcip_cfg.h \
    ../third_party/Profinet/src/source/psi/src/pnstack/tcip/tcip_inc.h \
    ../third_party/Profinet/src/source/sock/src/common/sock_low.h \
    ../third_party/Profinet/src/source/sock/src/common/sock_pls.h \
    ../third_party/Profinet/src/source/sock/src/common/sock_sys.h \
    ../third_party/Profinet/src/source/sock/src/common/sock_trc.h \
    ../third_party/Profinet/src/source/sock/src/common/sock_usr.h \
    ../third_party/Profinet/src/source/sock/src/inc/sock_int.h \
    ../third_party/Profinet/src/source/tcip/src/common/interniche_usr.h \
    ../third_party/Profinet/src/source/tcip/src/common/tcip_pls.h \
    ../third_party/Profinet/src/source/tcip/src/common/tcip_sys.h \
    ../third_party/Profinet/src/source/tcip/src/common/tcip_usr.h \
    ../third_party/Profinet/src/source/tcip/src/inc/tcip_int.h \
    ../third_party/Profinet/src/source/tcip/src/inc/tcip_low.h \
    ../third_party/Profinet/src/source/tcip/src/inc/tcip_trc.h \
    ../third_party/Profinet/src/source/tcip/src/inc/tcip_var.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/allports/ipport.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/allports/libport.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/allports/tcpport.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/arp.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/bsdsock.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/ether.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/icmp.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/in_utils.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/intimers.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/ip.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/ip6.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/mbuf.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/net.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/netbuf.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/nptcp.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/nptypes.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/pmtu.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/profiler.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/q.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/rfc1213_.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/snmp_vie.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/snmpport.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/sockcall.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/socket.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/socket6.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/sockvar.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/syslog.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/tcp.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/tcpapp.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/h/udp.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/ip/ip_reasm.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/ipmc/igmp.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/ipmc/igmp2.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/ipmc/igmp_cmn.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/snmp/asn1.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/snmp/npsnmp.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/snmp/parse.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/snmp/snmp_imp.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/snmp/snmp_var.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/tcp/in_pcb.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/tcp/protosw.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/tcp/tcp_fsm.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/tcp/tcp_seq.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/tcp/tcp_timr.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/tcp/tcp_var.h \
    ../third_party/Profinet/src/source/tcip/src_iniche_core/tcp/tcpip.h \
    dialogabout.h \
    dialogthrusterdiagi.h \
    dialogthrusterdiagii.h \
    dialogthrusterdiagiii.h \
    dialog6dof.h \
    csvstream.h \
    vesselshape.h




FORMS += \
        mainwindow.ui \
        display2ddialog.ui \
        dialogsetparameter.ui \
        dialogfixedsetpoint.ui \
        dialogstraightline.ui \
        dialogrotation.ui \
        dialogcooperation.ui \
    dialogabout.ui \
    dialogthrusterdiagi.ui \
    dialogthrusterdiagii.ui \
    dialogthrusterdiagiii.ui \
    dialog6dof.ui
