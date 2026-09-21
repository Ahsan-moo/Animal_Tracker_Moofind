export
APPZONE_LIB=$(APPZONE_DIR)\lib
APPZONE_BIN=${eclipse_home}\\plugins\com.appzonec.plugin.prebuilt_$(SDK_VERSION)\prebuilt\bin
OUTOBJDIR=obj
LOGONSERVER= 
TOOLCHAIN_PATH=${eclipse_home}\\plugins\com.telit.appzonec.toolchain.plugin.gccARMv6_493_4.9.3
APPZONE_DIR=${eclipse_home}\\plugins\com.telit.appzonec.plugin.me910g1_${FW_VERSION}_${PLUGIN_VERSION}
FW_VERSION=37_00_XX5
PLUGIN_VERSION=0.5.0.372222
SDK_VERSION=5.2.0
APPZONE_MAKEFILE_COMMON=${eclipse_home}\\plugins\com.appzonec.plugin_$(SDK_VERSION)\makefiles
AZ_STATIC_LIB=FALSE
APPZONE_INC=$(APPZONE_DIR)\m2m_inc
eclipse_home=C:\ProgramData\Telit\IoT_AppZone_IDE_5x\eclipse
TOOLCHAIN_BIN=${eclipse_home}\\plugins\com.telit.appzonec.toolchain.plugin.gccARMv6_493_4.9.3\arm_gcc493/bin/
AZ_BASE_MAKEFILE=az_makefile.mk
APPZONE_MAKEFILE=$(APPZONE_DIR)\makefiles
LIB_PATH=-L "${eclipse_home}\\plugins\com.telit.appzonec.toolchain.plugin.gccARMv6_493_4.9.3\arm_gcc493/lib/gcc/arm-none-eabi/4.9.3" -L "${eclipse_home}\\plugins\com.telit.appzonec.toolchain.plugin.gccARMv6_493_4.9.3\arm_gcc493/arm-none-eabi/lib" 
