#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/fast-buggy17.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/fast-buggy17.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=Ultrasound.c SystemClock.c Motors.c MillisecondTimer.c eeprom.c Wireless.c BatteryLib/ds2781.c BatteryLib/OneWire.c FastBuggy17_Main_Code_TUNING.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/Ultrasound.p1 ${OBJECTDIR}/SystemClock.p1 ${OBJECTDIR}/Motors.p1 ${OBJECTDIR}/MillisecondTimer.p1 ${OBJECTDIR}/eeprom.p1 ${OBJECTDIR}/Wireless.p1 ${OBJECTDIR}/BatteryLib/ds2781.p1 ${OBJECTDIR}/BatteryLib/OneWire.p1 ${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1
POSSIBLE_DEPFILES=${OBJECTDIR}/Ultrasound.p1.d ${OBJECTDIR}/SystemClock.p1.d ${OBJECTDIR}/Motors.p1.d ${OBJECTDIR}/MillisecondTimer.p1.d ${OBJECTDIR}/eeprom.p1.d ${OBJECTDIR}/Wireless.p1.d ${OBJECTDIR}/BatteryLib/ds2781.p1.d ${OBJECTDIR}/BatteryLib/OneWire.p1.d ${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1.d

# Object Files
OBJECTFILES=${OBJECTDIR}/Ultrasound.p1 ${OBJECTDIR}/SystemClock.p1 ${OBJECTDIR}/Motors.p1 ${OBJECTDIR}/MillisecondTimer.p1 ${OBJECTDIR}/eeprom.p1 ${OBJECTDIR}/Wireless.p1 ${OBJECTDIR}/BatteryLib/ds2781.p1 ${OBJECTDIR}/BatteryLib/OneWire.p1 ${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1

# Source Files
SOURCEFILES=Ultrasound.c SystemClock.c Motors.c MillisecondTimer.c eeprom.c Wireless.c BatteryLib/ds2781.c BatteryLib/OneWire.c FastBuggy17_Main_Code_TUNING.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/fast-buggy17.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F8722
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/Ultrasound.p1: Ultrasound.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Ultrasound.p1.d 
	@${RM} ${OBJECTDIR}/Ultrasound.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=pickit3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/Ultrasound.p1  Ultrasound.c 
	@-${MV} ${OBJECTDIR}/Ultrasound.d ${OBJECTDIR}/Ultrasound.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Ultrasound.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/SystemClock.p1: SystemClock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SystemClock.p1.d 
	@${RM} ${OBJECTDIR}/SystemClock.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=pickit3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/SystemClock.p1  SystemClock.c 
	@-${MV} ${OBJECTDIR}/SystemClock.d ${OBJECTDIR}/SystemClock.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/SystemClock.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Motors.p1: Motors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Motors.p1.d 
	@${RM} ${OBJECTDIR}/Motors.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=pickit3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/Motors.p1  Motors.c 
	@-${MV} ${OBJECTDIR}/Motors.d ${OBJECTDIR}/Motors.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Motors.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MillisecondTimer.p1: MillisecondTimer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MillisecondTimer.p1.d 
	@${RM} ${OBJECTDIR}/MillisecondTimer.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=pickit3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/MillisecondTimer.p1  MillisecondTimer.c 
	@-${MV} ${OBJECTDIR}/MillisecondTimer.d ${OBJECTDIR}/MillisecondTimer.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MillisecondTimer.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/eeprom.p1: eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/eeprom.p1.d 
	@${RM} ${OBJECTDIR}/eeprom.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=pickit3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/eeprom.p1  eeprom.c 
	@-${MV} ${OBJECTDIR}/eeprom.d ${OBJECTDIR}/eeprom.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/eeprom.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Wireless.p1: Wireless.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Wireless.p1.d 
	@${RM} ${OBJECTDIR}/Wireless.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=pickit3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/Wireless.p1  Wireless.c 
	@-${MV} ${OBJECTDIR}/Wireless.d ${OBJECTDIR}/Wireless.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Wireless.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/BatteryLib/ds2781.p1: BatteryLib/ds2781.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/BatteryLib" 
	@${RM} ${OBJECTDIR}/BatteryLib/ds2781.p1.d 
	@${RM} ${OBJECTDIR}/BatteryLib/ds2781.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=pickit3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/BatteryLib/ds2781.p1  BatteryLib/ds2781.c 
	@-${MV} ${OBJECTDIR}/BatteryLib/ds2781.d ${OBJECTDIR}/BatteryLib/ds2781.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/BatteryLib/ds2781.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/BatteryLib/OneWire.p1: BatteryLib/OneWire.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/BatteryLib" 
	@${RM} ${OBJECTDIR}/BatteryLib/OneWire.p1.d 
	@${RM} ${OBJECTDIR}/BatteryLib/OneWire.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=pickit3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/BatteryLib/OneWire.p1  BatteryLib/OneWire.c 
	@-${MV} ${OBJECTDIR}/BatteryLib/OneWire.d ${OBJECTDIR}/BatteryLib/OneWire.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/BatteryLib/OneWire.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1: FastBuggy17_Main_Code_TUNING.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1.d 
	@${RM} ${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=pickit3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1  FastBuggy17_Main_Code_TUNING.c 
	@-${MV} ${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.d ${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/Ultrasound.p1: Ultrasound.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Ultrasound.p1.d 
	@${RM} ${OBJECTDIR}/Ultrasound.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/Ultrasound.p1  Ultrasound.c 
	@-${MV} ${OBJECTDIR}/Ultrasound.d ${OBJECTDIR}/Ultrasound.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Ultrasound.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/SystemClock.p1: SystemClock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SystemClock.p1.d 
	@${RM} ${OBJECTDIR}/SystemClock.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/SystemClock.p1  SystemClock.c 
	@-${MV} ${OBJECTDIR}/SystemClock.d ${OBJECTDIR}/SystemClock.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/SystemClock.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Motors.p1: Motors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Motors.p1.d 
	@${RM} ${OBJECTDIR}/Motors.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/Motors.p1  Motors.c 
	@-${MV} ${OBJECTDIR}/Motors.d ${OBJECTDIR}/Motors.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Motors.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MillisecondTimer.p1: MillisecondTimer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MillisecondTimer.p1.d 
	@${RM} ${OBJECTDIR}/MillisecondTimer.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/MillisecondTimer.p1  MillisecondTimer.c 
	@-${MV} ${OBJECTDIR}/MillisecondTimer.d ${OBJECTDIR}/MillisecondTimer.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MillisecondTimer.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/eeprom.p1: eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/eeprom.p1.d 
	@${RM} ${OBJECTDIR}/eeprom.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/eeprom.p1  eeprom.c 
	@-${MV} ${OBJECTDIR}/eeprom.d ${OBJECTDIR}/eeprom.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/eeprom.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Wireless.p1: Wireless.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Wireless.p1.d 
	@${RM} ${OBJECTDIR}/Wireless.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/Wireless.p1  Wireless.c 
	@-${MV} ${OBJECTDIR}/Wireless.d ${OBJECTDIR}/Wireless.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Wireless.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/BatteryLib/ds2781.p1: BatteryLib/ds2781.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/BatteryLib" 
	@${RM} ${OBJECTDIR}/BatteryLib/ds2781.p1.d 
	@${RM} ${OBJECTDIR}/BatteryLib/ds2781.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/BatteryLib/ds2781.p1  BatteryLib/ds2781.c 
	@-${MV} ${OBJECTDIR}/BatteryLib/ds2781.d ${OBJECTDIR}/BatteryLib/ds2781.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/BatteryLib/ds2781.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/BatteryLib/OneWire.p1: BatteryLib/OneWire.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/BatteryLib" 
	@${RM} ${OBJECTDIR}/BatteryLib/OneWire.p1.d 
	@${RM} ${OBJECTDIR}/BatteryLib/OneWire.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/BatteryLib/OneWire.p1  BatteryLib/OneWire.c 
	@-${MV} ${OBJECTDIR}/BatteryLib/OneWire.d ${OBJECTDIR}/BatteryLib/OneWire.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/BatteryLib/OneWire.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1: FastBuggy17_Main_Code_TUNING.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1.d 
	@${RM} ${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib $(COMPARISON_BUILD)  --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1  FastBuggy17_Main_Code_TUNING.c 
	@-${MV} ${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.d ${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/FastBuggy17_Main_Code_TUNING.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/fast-buggy17.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) --chip=$(MP_PROCESSOR_OPTION) -G -mdist/${CND_CONF}/${IMAGE_TYPE}/fast-buggy17.${IMAGE_TYPE}.map  -D__DEBUG=1 --debugger=pickit3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     --rom=default,-1fd30-1ffff --ram=default,-ef4-eff,-f9c-f9c,-fd4-fd4,-fdb-fdf,-fe3-fe7,-feb-fef,-ffd-fff  $(COMPARISON_BUILD) --memorysummary dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -odist/${CND_CONF}/${IMAGE_TYPE}/fast-buggy17.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	@${RM} dist/${CND_CONF}/${IMAGE_TYPE}/fast-buggy17.${IMAGE_TYPE}.hex 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/fast-buggy17.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) --chip=$(MP_PROCESSOR_OPTION) -G -mdist/${CND_CONF}/${IMAGE_TYPE}/fast-buggy17.${IMAGE_TYPE}.map  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     $(COMPARISON_BUILD) --memorysummary dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -odist/${CND_CONF}/${IMAGE_TYPE}/fast-buggy17.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
