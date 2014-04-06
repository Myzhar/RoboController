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
ifeq "$(wildcard nbproject/Makefile-local-Myzhar.mk)" "nbproject/Makefile-local-Myzhar.mk"
include nbproject/Makefile-local-Myzhar.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=Myzhar
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/CPU.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/CPU.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=src/adc.c src/main.c src/modbus.c src/pid.c src/settings.c src/sw_timer.c src/uart.c src/Eeprom.c src/Alarm.c src/Led.c src/motor.c src/DEE_Emulation_16-bit.c src/Flash_Operations.s

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/src/adc.o ${OBJECTDIR}/src/main.o ${OBJECTDIR}/src/modbus.o ${OBJECTDIR}/src/pid.o ${OBJECTDIR}/src/settings.o ${OBJECTDIR}/src/sw_timer.o ${OBJECTDIR}/src/uart.o ${OBJECTDIR}/src/Eeprom.o ${OBJECTDIR}/src/Alarm.o ${OBJECTDIR}/src/Led.o ${OBJECTDIR}/src/motor.o ${OBJECTDIR}/src/DEE_Emulation_16-bit.o ${OBJECTDIR}/src/Flash_Operations.o
POSSIBLE_DEPFILES=${OBJECTDIR}/src/adc.o.d ${OBJECTDIR}/src/main.o.d ${OBJECTDIR}/src/modbus.o.d ${OBJECTDIR}/src/pid.o.d ${OBJECTDIR}/src/settings.o.d ${OBJECTDIR}/src/sw_timer.o.d ${OBJECTDIR}/src/uart.o.d ${OBJECTDIR}/src/Eeprom.o.d ${OBJECTDIR}/src/Alarm.o.d ${OBJECTDIR}/src/Led.o.d ${OBJECTDIR}/src/motor.o.d ${OBJECTDIR}/src/DEE_Emulation_16-bit.o.d ${OBJECTDIR}/src/Flash_Operations.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/src/adc.o ${OBJECTDIR}/src/main.o ${OBJECTDIR}/src/modbus.o ${OBJECTDIR}/src/pid.o ${OBJECTDIR}/src/settings.o ${OBJECTDIR}/src/sw_timer.o ${OBJECTDIR}/src/uart.o ${OBJECTDIR}/src/Eeprom.o ${OBJECTDIR}/src/Alarm.o ${OBJECTDIR}/src/Led.o ${OBJECTDIR}/src/motor.o ${OBJECTDIR}/src/DEE_Emulation_16-bit.o ${OBJECTDIR}/src/Flash_Operations.o

# Source Files
SOURCEFILES=src/adc.c src/main.c src/modbus.c src/pid.c src/settings.c src/sw_timer.c src/uart.c src/Eeprom.c src/Alarm.c src/Led.c src/motor.c src/DEE_Emulation_16-bit.c src/Flash_Operations.s


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
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-Myzhar.mk dist/${CND_CONF}/${IMAGE_TYPE}/CPU.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33FJ128MC804
MP_LINKER_FILE_OPTION=,--script=p33FJ128MC804.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/src/adc.o: src/adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/adc.o.d 
	@${RM} ${OBJECTDIR}/src/adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/adc.c  -o ${OBJECTDIR}/src/adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/adc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/adc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/main.o: src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/main.o.d 
	@${RM} ${OBJECTDIR}/src/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/main.c  -o ${OBJECTDIR}/src/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/modbus.o: src/modbus.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/modbus.o.d 
	@${RM} ${OBJECTDIR}/src/modbus.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/modbus.c  -o ${OBJECTDIR}/src/modbus.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/modbus.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/modbus.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/pid.o: src/pid.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/pid.o.d 
	@${RM} ${OBJECTDIR}/src/pid.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/pid.c  -o ${OBJECTDIR}/src/pid.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/pid.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/pid.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/settings.o: src/settings.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/settings.o.d 
	@${RM} ${OBJECTDIR}/src/settings.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/settings.c  -o ${OBJECTDIR}/src/settings.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/settings.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/settings.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/sw_timer.o: src/sw_timer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/sw_timer.o.d 
	@${RM} ${OBJECTDIR}/src/sw_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/sw_timer.c  -o ${OBJECTDIR}/src/sw_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/sw_timer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/sw_timer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/uart.o: src/uart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/uart.o.d 
	@${RM} ${OBJECTDIR}/src/uart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/uart.c  -o ${OBJECTDIR}/src/uart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/uart.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/uart.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Eeprom.o: src/Eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/Eeprom.o.d 
	@${RM} ${OBJECTDIR}/src/Eeprom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Eeprom.c  -o ${OBJECTDIR}/src/Eeprom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Eeprom.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/Eeprom.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Alarm.o: src/Alarm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/Alarm.o.d 
	@${RM} ${OBJECTDIR}/src/Alarm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Alarm.c  -o ${OBJECTDIR}/src/Alarm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Alarm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/Alarm.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Led.o: src/Led.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/Led.o.d 
	@${RM} ${OBJECTDIR}/src/Led.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Led.c  -o ${OBJECTDIR}/src/Led.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Led.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/Led.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/motor.o: src/motor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/motor.o.d 
	@${RM} ${OBJECTDIR}/src/motor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/motor.c  -o ${OBJECTDIR}/src/motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/motor.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/motor.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/DEE_Emulation_16-bit.o: src/DEE_Emulation_16-bit.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/DEE_Emulation_16-bit.o.d 
	@${RM} ${OBJECTDIR}/src/DEE_Emulation_16-bit.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/DEE_Emulation_16-bit.c  -o ${OBJECTDIR}/src/DEE_Emulation_16-bit.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/DEE_Emulation_16-bit.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/DEE_Emulation_16-bit.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/src/adc.o: src/adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/adc.o.d 
	@${RM} ${OBJECTDIR}/src/adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/adc.c  -o ${OBJECTDIR}/src/adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/adc.o.d"      -g -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/adc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/main.o: src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/main.o.d 
	@${RM} ${OBJECTDIR}/src/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/main.c  -o ${OBJECTDIR}/src/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/main.o.d"      -g -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/modbus.o: src/modbus.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/modbus.o.d 
	@${RM} ${OBJECTDIR}/src/modbus.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/modbus.c  -o ${OBJECTDIR}/src/modbus.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/modbus.o.d"      -g -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/modbus.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/pid.o: src/pid.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/pid.o.d 
	@${RM} ${OBJECTDIR}/src/pid.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/pid.c  -o ${OBJECTDIR}/src/pid.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/pid.o.d"      -g -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/pid.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/settings.o: src/settings.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/settings.o.d 
	@${RM} ${OBJECTDIR}/src/settings.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/settings.c  -o ${OBJECTDIR}/src/settings.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/settings.o.d"      -g -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/settings.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/sw_timer.o: src/sw_timer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/sw_timer.o.d 
	@${RM} ${OBJECTDIR}/src/sw_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/sw_timer.c  -o ${OBJECTDIR}/src/sw_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/sw_timer.o.d"      -g -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/sw_timer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/uart.o: src/uart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/uart.o.d 
	@${RM} ${OBJECTDIR}/src/uart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/uart.c  -o ${OBJECTDIR}/src/uart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/uart.o.d"      -g -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/uart.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Eeprom.o: src/Eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/Eeprom.o.d 
	@${RM} ${OBJECTDIR}/src/Eeprom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Eeprom.c  -o ${OBJECTDIR}/src/Eeprom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Eeprom.o.d"      -g -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/Eeprom.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Alarm.o: src/Alarm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/Alarm.o.d 
	@${RM} ${OBJECTDIR}/src/Alarm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Alarm.c  -o ${OBJECTDIR}/src/Alarm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Alarm.o.d"      -g -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/Alarm.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Led.o: src/Led.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/Led.o.d 
	@${RM} ${OBJECTDIR}/src/Led.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Led.c  -o ${OBJECTDIR}/src/Led.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Led.o.d"      -g -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/Led.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/motor.o: src/motor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/motor.o.d 
	@${RM} ${OBJECTDIR}/src/motor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/motor.c  -o ${OBJECTDIR}/src/motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/motor.o.d"      -g -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/motor.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/DEE_Emulation_16-bit.o: src/DEE_Emulation_16-bit.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/DEE_Emulation_16-bit.o.d 
	@${RM} ${OBJECTDIR}/src/DEE_Emulation_16-bit.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/DEE_Emulation_16-bit.c  -o ${OBJECTDIR}/src/DEE_Emulation_16-bit.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/DEE_Emulation_16-bit.o.d"      -g -omf=elf -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/DEE_Emulation_16-bit.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/src/Flash_Operations.o: src/Flash_Operations.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/Flash_Operations.o.d 
	@${RM} ${OBJECTDIR}/src/Flash_Operations.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  src/Flash_Operations.s  -o ${OBJECTDIR}/src/Flash_Operations.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -Wa,-MD,"${OBJECTDIR}/src/Flash_Operations.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-g,--no-relax,--keep-locals,-ahli$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/src/Flash_Operations.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/src/Flash_Operations.o: src/Flash_Operations.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/Flash_Operations.o.d 
	@${RM} ${OBJECTDIR}/src/Flash_Operations.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  src/Flash_Operations.s  -o ${OBJECTDIR}/src/Flash_Operations.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -Wa,-MD,"${OBJECTDIR}/src/Flash_Operations.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,--keep-locals,-ahli$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/src/Flash_Operations.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/CPU.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  lib/libc-elf.a lib/libdsp-elf.a lib/libfastm-elf.a lib/libq-dsp-elf.a lib/libq-elf.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/CPU.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    lib\libc-elf.a lib\libdsp-elf.a lib\libfastm-elf.a lib\libq-dsp-elf.a lib\libq-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf  -mreserve=data@0x800:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x825 -mreserve=data@0x826:0x84F   -Wl,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,$(MP_LINKER_FILE_OPTION),--heap=512,--check-sections,--data-init,--no-pack-data,--no-handles,--no-isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="file.map",--report-mem,--cref$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/CPU.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  lib/libc-elf.a lib/libdsp-elf.a lib/libfastm-elf.a lib/libq-dsp-elf.a lib/libq-elf.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/CPU.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    lib\libc-elf.a lib\libdsp-elf.a lib\libfastm-elf.a lib\libq-dsp-elf.a lib\libq-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -Wl,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--heap=512,--check-sections,--data-init,--no-pack-data,--no-handles,--no-isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="file.map",--report-mem,--cref$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/CPU.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/Myzhar
	${RM} -r dist/Myzhar

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
