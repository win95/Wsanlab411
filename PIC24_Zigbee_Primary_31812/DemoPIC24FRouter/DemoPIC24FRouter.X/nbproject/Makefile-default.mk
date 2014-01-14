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
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/DemoPIC24FRouter.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/DemoPIC24FRouter.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common/delay.c ../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common/SHT1x.c ../../Microchip/Common/sralloc.c ../../Microchip/Common/Console.c ../../Microchip/Common/MSPI.c ../../Microchip/ZigBeeStack/zZDO.C ../../Microchip/ZigBeeStack/SymbolTime.c ../../Microchip/ZigBeeStack/zAPS.c ../../Microchip/ZigBeeStack/ZigbeeTasks.c ../../Microchip/ZigBeeStack/zMAC_MRF24J40.c ../../Microchip/ZigBeeStack/zNVM.c ../../Microchip/ZigBeeStack/zNWK.c ../../Microchip/ZigBeeStack/zPHY_MRF24J40.c ../../Microchip/ZigBeeStack/zSecurity_MRF24J40.c ../myZigBee.c ../Router.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1731036568/delay.o ${OBJECTDIR}/_ext/1731036568/SHT1x.o ${OBJECTDIR}/_ext/385479254/sralloc.o ${OBJECTDIR}/_ext/385479254/Console.o ${OBJECTDIR}/_ext/385479254/MSPI.o ${OBJECTDIR}/_ext/1163520513/zZDO.o ${OBJECTDIR}/_ext/1163520513/SymbolTime.o ${OBJECTDIR}/_ext/1163520513/zAPS.o ${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o ${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o ${OBJECTDIR}/_ext/1163520513/zNVM.o ${OBJECTDIR}/_ext/1163520513/zNWK.o ${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o ${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o ${OBJECTDIR}/_ext/1472/myZigBee.o ${OBJECTDIR}/_ext/1472/Router.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1731036568/delay.o.d ${OBJECTDIR}/_ext/1731036568/SHT1x.o.d ${OBJECTDIR}/_ext/385479254/sralloc.o.d ${OBJECTDIR}/_ext/385479254/Console.o.d ${OBJECTDIR}/_ext/385479254/MSPI.o.d ${OBJECTDIR}/_ext/1163520513/zZDO.o.d ${OBJECTDIR}/_ext/1163520513/SymbolTime.o.d ${OBJECTDIR}/_ext/1163520513/zAPS.o.d ${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o.d ${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o.d ${OBJECTDIR}/_ext/1163520513/zNVM.o.d ${OBJECTDIR}/_ext/1163520513/zNWK.o.d ${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o.d ${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o.d ${OBJECTDIR}/_ext/1472/myZigBee.o.d ${OBJECTDIR}/_ext/1472/Router.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1731036568/delay.o ${OBJECTDIR}/_ext/1731036568/SHT1x.o ${OBJECTDIR}/_ext/385479254/sralloc.o ${OBJECTDIR}/_ext/385479254/Console.o ${OBJECTDIR}/_ext/385479254/MSPI.o ${OBJECTDIR}/_ext/1163520513/zZDO.o ${OBJECTDIR}/_ext/1163520513/SymbolTime.o ${OBJECTDIR}/_ext/1163520513/zAPS.o ${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o ${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o ${OBJECTDIR}/_ext/1163520513/zNVM.o ${OBJECTDIR}/_ext/1163520513/zNWK.o ${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o ${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o ${OBJECTDIR}/_ext/1472/myZigBee.o ${OBJECTDIR}/_ext/1472/Router.o

# Source Files
SOURCEFILES=../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common/delay.c ../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common/SHT1x.c ../../Microchip/Common/sralloc.c ../../Microchip/Common/Console.c ../../Microchip/Common/MSPI.c ../../Microchip/ZigBeeStack/zZDO.C ../../Microchip/ZigBeeStack/SymbolTime.c ../../Microchip/ZigBeeStack/zAPS.c ../../Microchip/ZigBeeStack/ZigbeeTasks.c ../../Microchip/ZigBeeStack/zMAC_MRF24J40.c ../../Microchip/ZigBeeStack/zNVM.c ../../Microchip/ZigBeeStack/zNWK.c ../../Microchip/ZigBeeStack/zPHY_MRF24J40.c ../../Microchip/ZigBeeStack/zSecurity_MRF24J40.c ../myZigBee.c ../Router.c


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
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/DemoPIC24FRouter.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24FJ128GA306
MP_LINKER_FILE_OPTION=,-Tp24FJ128GA306.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1731036568/delay.o: ../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common/delay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1731036568 
	@${RM} ${OBJECTDIR}/_ext/1731036568/delay.o.d 
	@${RM} ${OBJECTDIR}/_ext/1731036568/delay.o.ok ${OBJECTDIR}/_ext/1731036568/delay.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1731036568/delay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1731036568/delay.o.d" -o ${OBJECTDIR}/_ext/1731036568/delay.o ../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common/delay.c    
	
${OBJECTDIR}/_ext/1731036568/SHT1x.o: ../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common/SHT1x.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1731036568 
	@${RM} ${OBJECTDIR}/_ext/1731036568/SHT1x.o.d 
	@${RM} ${OBJECTDIR}/_ext/1731036568/SHT1x.o.ok ${OBJECTDIR}/_ext/1731036568/SHT1x.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1731036568/SHT1x.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1731036568/SHT1x.o.d" -o ${OBJECTDIR}/_ext/1731036568/SHT1x.o ../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common/SHT1x.c    
	
${OBJECTDIR}/_ext/385479254/sralloc.o: ../../Microchip/Common/sralloc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/385479254 
	@${RM} ${OBJECTDIR}/_ext/385479254/sralloc.o.d 
	@${RM} ${OBJECTDIR}/_ext/385479254/sralloc.o.ok ${OBJECTDIR}/_ext/385479254/sralloc.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/385479254/sralloc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/385479254/sralloc.o.d" -o ${OBJECTDIR}/_ext/385479254/sralloc.o ../../Microchip/Common/sralloc.c    
	
${OBJECTDIR}/_ext/385479254/Console.o: ../../Microchip/Common/Console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/385479254 
	@${RM} ${OBJECTDIR}/_ext/385479254/Console.o.d 
	@${RM} ${OBJECTDIR}/_ext/385479254/Console.o.ok ${OBJECTDIR}/_ext/385479254/Console.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/385479254/Console.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/385479254/Console.o.d" -o ${OBJECTDIR}/_ext/385479254/Console.o ../../Microchip/Common/Console.c    
	
${OBJECTDIR}/_ext/385479254/MSPI.o: ../../Microchip/Common/MSPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/385479254 
	@${RM} ${OBJECTDIR}/_ext/385479254/MSPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/385479254/MSPI.o.ok ${OBJECTDIR}/_ext/385479254/MSPI.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/385479254/MSPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/385479254/MSPI.o.d" -o ${OBJECTDIR}/_ext/385479254/MSPI.o ../../Microchip/Common/MSPI.c    
	
${OBJECTDIR}/_ext/1163520513/zZDO.o: ../../Microchip/ZigBeeStack/zZDO.C  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zZDO.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zZDO.o.ok ${OBJECTDIR}/_ext/1163520513/zZDO.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zZDO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zZDO.o.d" -o ${OBJECTDIR}/_ext/1163520513/zZDO.o ../../Microchip/ZigBeeStack/zZDO.C    
	
${OBJECTDIR}/_ext/1163520513/SymbolTime.o: ../../Microchip/ZigBeeStack/SymbolTime.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/SymbolTime.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/SymbolTime.o.ok ${OBJECTDIR}/_ext/1163520513/SymbolTime.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/SymbolTime.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/SymbolTime.o.d" -o ${OBJECTDIR}/_ext/1163520513/SymbolTime.o ../../Microchip/ZigBeeStack/SymbolTime.c    
	
${OBJECTDIR}/_ext/1163520513/zAPS.o: ../../Microchip/ZigBeeStack/zAPS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zAPS.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zAPS.o.ok ${OBJECTDIR}/_ext/1163520513/zAPS.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zAPS.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zAPS.o.d" -o ${OBJECTDIR}/_ext/1163520513/zAPS.o ../../Microchip/ZigBeeStack/zAPS.c    
	
${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o: ../../Microchip/ZigBeeStack/ZigbeeTasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o.ok ${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o.d" -o ${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o ../../Microchip/ZigBeeStack/ZigbeeTasks.c    
	
${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o: ../../Microchip/ZigBeeStack/zMAC_MRF24J40.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o.ok ${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o.d" -o ${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o ../../Microchip/ZigBeeStack/zMAC_MRF24J40.c    
	
${OBJECTDIR}/_ext/1163520513/zNVM.o: ../../Microchip/ZigBeeStack/zNVM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zNVM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zNVM.o.ok ${OBJECTDIR}/_ext/1163520513/zNVM.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zNVM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zNVM.o.d" -o ${OBJECTDIR}/_ext/1163520513/zNVM.o ../../Microchip/ZigBeeStack/zNVM.c    
	
${OBJECTDIR}/_ext/1163520513/zNWK.o: ../../Microchip/ZigBeeStack/zNWK.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zNWK.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zNWK.o.ok ${OBJECTDIR}/_ext/1163520513/zNWK.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zNWK.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zNWK.o.d" -o ${OBJECTDIR}/_ext/1163520513/zNWK.o ../../Microchip/ZigBeeStack/zNWK.c    
	
${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o: ../../Microchip/ZigBeeStack/zPHY_MRF24J40.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o.ok ${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o.d" -o ${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o ../../Microchip/ZigBeeStack/zPHY_MRF24J40.c    
	
${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o: ../../Microchip/ZigBeeStack/zSecurity_MRF24J40.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o.ok ${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o.d" -o ${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o ../../Microchip/ZigBeeStack/zSecurity_MRF24J40.c    
	
${OBJECTDIR}/_ext/1472/myZigBee.o: ../myZigBee.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/myZigBee.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/myZigBee.o.ok ${OBJECTDIR}/_ext/1472/myZigBee.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/myZigBee.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1472/myZigBee.o.d" -o ${OBJECTDIR}/_ext/1472/myZigBee.o ../myZigBee.c    
	
${OBJECTDIR}/_ext/1472/Router.o: ../Router.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/Router.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/Router.o.ok ${OBJECTDIR}/_ext/1472/Router.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/Router.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1472/Router.o.d" -o ${OBJECTDIR}/_ext/1472/Router.o ../Router.c    
	
else
${OBJECTDIR}/_ext/1731036568/delay.o: ../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common/delay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1731036568 
	@${RM} ${OBJECTDIR}/_ext/1731036568/delay.o.d 
	@${RM} ${OBJECTDIR}/_ext/1731036568/delay.o.ok ${OBJECTDIR}/_ext/1731036568/delay.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1731036568/delay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1731036568/delay.o.d" -o ${OBJECTDIR}/_ext/1731036568/delay.o ../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common/delay.c    
	
${OBJECTDIR}/_ext/1731036568/SHT1x.o: ../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common/SHT1x.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1731036568 
	@${RM} ${OBJECTDIR}/_ext/1731036568/SHT1x.o.d 
	@${RM} ${OBJECTDIR}/_ext/1731036568/SHT1x.o.ok ${OBJECTDIR}/_ext/1731036568/SHT1x.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1731036568/SHT1x.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1731036568/SHT1x.o.d" -o ${OBJECTDIR}/_ext/1731036568/SHT1x.o ../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common/SHT1x.c    
	
${OBJECTDIR}/_ext/385479254/sralloc.o: ../../Microchip/Common/sralloc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/385479254 
	@${RM} ${OBJECTDIR}/_ext/385479254/sralloc.o.d 
	@${RM} ${OBJECTDIR}/_ext/385479254/sralloc.o.ok ${OBJECTDIR}/_ext/385479254/sralloc.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/385479254/sralloc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/385479254/sralloc.o.d" -o ${OBJECTDIR}/_ext/385479254/sralloc.o ../../Microchip/Common/sralloc.c    
	
${OBJECTDIR}/_ext/385479254/Console.o: ../../Microchip/Common/Console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/385479254 
	@${RM} ${OBJECTDIR}/_ext/385479254/Console.o.d 
	@${RM} ${OBJECTDIR}/_ext/385479254/Console.o.ok ${OBJECTDIR}/_ext/385479254/Console.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/385479254/Console.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/385479254/Console.o.d" -o ${OBJECTDIR}/_ext/385479254/Console.o ../../Microchip/Common/Console.c    
	
${OBJECTDIR}/_ext/385479254/MSPI.o: ../../Microchip/Common/MSPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/385479254 
	@${RM} ${OBJECTDIR}/_ext/385479254/MSPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/385479254/MSPI.o.ok ${OBJECTDIR}/_ext/385479254/MSPI.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/385479254/MSPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/385479254/MSPI.o.d" -o ${OBJECTDIR}/_ext/385479254/MSPI.o ../../Microchip/Common/MSPI.c    
	
${OBJECTDIR}/_ext/1163520513/zZDO.o: ../../Microchip/ZigBeeStack/zZDO.C  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zZDO.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zZDO.o.ok ${OBJECTDIR}/_ext/1163520513/zZDO.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zZDO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zZDO.o.d" -o ${OBJECTDIR}/_ext/1163520513/zZDO.o ../../Microchip/ZigBeeStack/zZDO.C    
	
${OBJECTDIR}/_ext/1163520513/SymbolTime.o: ../../Microchip/ZigBeeStack/SymbolTime.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/SymbolTime.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/SymbolTime.o.ok ${OBJECTDIR}/_ext/1163520513/SymbolTime.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/SymbolTime.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/SymbolTime.o.d" -o ${OBJECTDIR}/_ext/1163520513/SymbolTime.o ../../Microchip/ZigBeeStack/SymbolTime.c    
	
${OBJECTDIR}/_ext/1163520513/zAPS.o: ../../Microchip/ZigBeeStack/zAPS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zAPS.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zAPS.o.ok ${OBJECTDIR}/_ext/1163520513/zAPS.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zAPS.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zAPS.o.d" -o ${OBJECTDIR}/_ext/1163520513/zAPS.o ../../Microchip/ZigBeeStack/zAPS.c    
	
${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o: ../../Microchip/ZigBeeStack/ZigbeeTasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o.ok ${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o.d" -o ${OBJECTDIR}/_ext/1163520513/ZigbeeTasks.o ../../Microchip/ZigBeeStack/ZigbeeTasks.c    
	
${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o: ../../Microchip/ZigBeeStack/zMAC_MRF24J40.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o.ok ${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o.d" -o ${OBJECTDIR}/_ext/1163520513/zMAC_MRF24J40.o ../../Microchip/ZigBeeStack/zMAC_MRF24J40.c    
	
${OBJECTDIR}/_ext/1163520513/zNVM.o: ../../Microchip/ZigBeeStack/zNVM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zNVM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zNVM.o.ok ${OBJECTDIR}/_ext/1163520513/zNVM.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zNVM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zNVM.o.d" -o ${OBJECTDIR}/_ext/1163520513/zNVM.o ../../Microchip/ZigBeeStack/zNVM.c    
	
${OBJECTDIR}/_ext/1163520513/zNWK.o: ../../Microchip/ZigBeeStack/zNWK.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zNWK.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zNWK.o.ok ${OBJECTDIR}/_ext/1163520513/zNWK.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zNWK.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zNWK.o.d" -o ${OBJECTDIR}/_ext/1163520513/zNWK.o ../../Microchip/ZigBeeStack/zNWK.c    
	
${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o: ../../Microchip/ZigBeeStack/zPHY_MRF24J40.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o.ok ${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o.d" -o ${OBJECTDIR}/_ext/1163520513/zPHY_MRF24J40.o ../../Microchip/ZigBeeStack/zPHY_MRF24J40.c    
	
${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o: ../../Microchip/ZigBeeStack/zSecurity_MRF24J40.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163520513 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o.ok ${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o.d" -o ${OBJECTDIR}/_ext/1163520513/zSecurity_MRF24J40.o ../../Microchip/ZigBeeStack/zSecurity_MRF24J40.c    
	
${OBJECTDIR}/_ext/1472/myZigBee.o: ../myZigBee.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/myZigBee.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/myZigBee.o.ok ${OBJECTDIR}/_ext/1472/myZigBee.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/myZigBee.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1472/myZigBee.o.d" -o ${OBJECTDIR}/_ext/1472/myZigBee.o ../myZigBee.c    
	
${OBJECTDIR}/_ext/1472/Router.o: ../Router.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/Router.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/Router.o.ok ${OBJECTDIR}/_ext/1472/Router.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/Router.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -I"../../microchip/ZigBeeStack" -I"../../../DemoPIC24FRouter" -I"../../../microchip/ZigBeeStack" -I"../../microchip/Common" -I"../../ZigBeeStack" -I".." -I"../../Common" -I"../../Microchip/Common" -I"../../Microchip/ZigBeeStack" -I"../../../../../../Desktop/Actor200913Dev/PIC24_Zigbee_Primary_31812/Microchip/Common" -O2 -MMD -MF "${OBJECTDIR}/_ext/1472/Router.o.d" -o ${OBJECTDIR}/_ext/1472/Router.o ../Router.c    
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/DemoPIC24FRouter.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG  -o dist/${CND_CONF}/${IMAGE_TYPE}/DemoPIC24FRouter.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,--heap=4090,-L"../../../../../../../../Program Files/Microchip/MPLAB C30/lib",-L".."$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1
else
dist/${CND_CONF}/${IMAGE_TYPE}/DemoPIC24FRouter.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/DemoPIC24FRouter.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,--heap=4090,-L"../../../../../../../../Program Files/Microchip/MPLAB C30/lib",-L".."$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION)
	${MP_CC_DIR}\\pic30-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/DemoPIC24FRouter.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -omf=elf
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
