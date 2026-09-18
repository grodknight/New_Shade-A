// Copyright (C) 2010 to the present, Crestron Electronics, Inc.
// All rights reserved.
// No part of this software may be reproduced in any form, machine
// or natural, without the express written consent of Crestron Electronics.
/**
 * \file    cm-applicationCallbacks.c
 * \brief   Common transport modules, application callbacks templates
 * \details Common communication transport module allows small targets to build Cresnet and infiNet versions by selecting
 *          a build or library for the desired communication medium.
 *
 *          Application callbacks are the means for the modules to communicate with the application when the module detects
 *          an event that needs to be reported to or acted upon by the application.  This file contains functions and skeleton
 *          code that needs to be implemented by the selected application to be able to use the transport module.
 *
 *          Except for instances with preprocessor-directive wrapping, all of the functions in this file need to be implemented
 *          or stubbed out if not relevant.  Empty functions are not a problem as they will be optimized out by the compiler
 *          anyway.  The functions do not have to be present in this file but must be extern visible to the overall project.
 *
 * \date    05/05/10
 **/

#include <stdlib.h>

#include PROJECT_HEADER

#include "common\common.h"
#include "common\cm-errorAndStatusCodes.h"    // used for CmStatusEventErrorCodes enumeration
#include "common\cm-errlog.h"
#include "common\cm-tsid.h"
#include "transport\cm-trans-resync.h"
#include "transport\cm-trans-fullparse.h"
#include "bsp_csm_mcu.h" 
#include "link\cm-link.h"

#include "product.h"
#include "system.h"
#include "eeprom\eeprom_AppCommon.h"
#include "event\event.h"
#include "frontpanel\button.h"
#include "frontpanel\frontpanel_tsid.h"
#include "frontpanel\led.h"

#include "cm-projectDefines.h"         // used for defining project/module build options
#include "csm_ctrl.h"
#include "csm_cresnet.h"
#include "csm_diag.h"
#include "csm_eepromTest.h"
#include "cm-trans-resync.h"


#include "shades\shade-debug.h"
#include "shades\shade-debug-console.h"
#include "shades\shade-debug-counters.h"
#include "shades\shade-rcb-fb.h"
#include "shades\shade-limits.h"
#include "shades\shade-posctrl.h"

#include "shades\shade-rcb-logging.h"

#ifdef __CNET_DEVICE__
    #include "cresnet\console.h"
    #include "cresnet\commands.h"
    #include "cresnet\download.h"
#else
    #include "lp-console.h"
    #include "lp-printf.h"
#endif

// for testing shade RCB output logging
#ifdef __CNET_DEVICE__
 #ifdef CSM2
  #include "AppCommon\shades\shade-rcb-logging.h"
 #endif 
#endif

// For SPI External SPI Flash Test
#ifdef __MNET_DEVICE__
 #if !defined(CSM_QMTDC_163)&&!defined(CSM_DRAPE_MOTOR)
   #include "selftest\selftest-common.h"
 #endif
#endif
 

#ifdef CSM2

    #include "csm2_misc.h"
    #include "csm_serial_comm.h"
	
    #ifdef __CNET_DEVICE__
       #include "bsp_csm_mcu.h"
    #endif
    
    #ifdef USE_MIDPOINT_CORRECTION
      #include "csm_shade_tracking.h"
    #endif 

    #ifdef CSM_DRAPE_MOTOR
     #include "csm_drape.h"
    #endif
   
#endif
 
//#ifdef CSM_QMTDC_163
 #include "csm2_silhouette.h"
//#endif


UINT8 slaveUpgradeReady = 0;
BOOL g_bGotCommandInRfrcon;
/*******************************************************************************
* sendReportCresnetOk - Function used to enable the reportcresnet from this device 
*                       when it has the slave succesfully upgraded 
*******************************************************************************/ 
void sendReportCresnetOk(void)
{
   slaveUpgradeReady = 1;
   CmLinkOnRespondEvent(CMSTA_LINK_RESPOND_IDCHANGE);
}

/* ----------------------------------------------------------------------------- */



 #if defined(CSM_QMTDC_163)||defined(CSM_DRAPE_MOTOR)
     #ifdef __MNET_DEVICE__
     
  //#warning Compiling the NVDATA functions for 1Nm EX and DRAPE EX  

  #include "eeprom/eeprom_SimPageStruct.h"
  #include "eeprom/eeprom_AppCommon.h"

  /* Added - Aug 2012 - Temp Buffers for RFRCON Console Printing use */
  static char CmdBuffer[50];
  //REMOVE TO SAVE SPACE::   static char NvBuffer[136]; //Commented because we are not printing the NV data
  static char NvBuffer[136]; //Commented because we are not printing the NV data

  //Prints out the NV Data/Simulated EEPROM Struct  
  static void NvData_Output_RamData( simeeprom_Block_Info_T * psBlk );
  static void NvData_Output_ConstData( simeeprom_Block_Info_T * psBlk );
  static void NvData_Output_DataLine( UINT8 * pnDataBlock, UINT8 byteCount );
  //REMOVE TO SAVE SPACE::   static void NvData_Output_NvPageData( simeeprom_Block_Info_T * psBlk );
  static void NvData_Output_NvPageData( simeeprom_Block_Info_T * psBlk );


/*******************************************************************************
* Function Name  : NvDataCommand
* Description    : Dumps NV storage data
* Input          : none
* Output         : none
* Return         : none
*******************************************************************************/
static void csm2_NvDataCommand( void )
{
    simeeprom_Block_Info_T sBlkData;    // for retrieving information about and data for eeprom storage
    UINT8 BLOCK = 0;                    // the count of blocks (data storage sets), seed 0 to start

    // initial greeting and basic info
    LpPrintString("\r\n");
    LpSPPrintf(NULL, "NV data storage info (%s mode)\r\n", simeeprom_IsStripeMode() ? "stripe" : "mirror" );

    // keep going until we dump all the blocks; end will be denoted with a fail to retrieve a structure
    while ( simeeprom_GetBlockStructure(BLOCK, &sBlkData) == TRUE )
    {
        // basic information about how the block is structured and sized
        LpSPPrintf(NULL, "\r\n Block %u data:\r\n", BLOCK );
        LpSPPrintf(NULL, " First/last page:    0x%2x/0x%2x \r\n", sBlkData.first_page, sBlkData.last_page );
        LpSPPrintf(NULL, " Page size/offset:   %u/%u \r\n", sBlkData.page_size, sBlkData.page_offset );
        LpSPPrintf(NULL, " Data size:          %u\r\n", (int)sBlkData.data_size );
        LpSPPrintf(NULL, " Version:            %u\r\n", sBlkData.version );
        LpSPPrintf(NULL, " Signature:          %u\r\n", sBlkData.signature );
        LpSPPrintf(NULL, " Current Page:       0x%2x\r\n", (*sBlkData.page) );
        
        // display the shadow ram for the variables stored
        LpPrintString( " RAM contents:\r\n" );
        NvData_Output_RamData(&sBlkData);

        // display the defaults for the variables stored
        LpPrintString( " Defaults contents:\r\n" );
        NvData_Output_ConstData(&sBlkData);

        // display the actual contents of the NV storage pages
        LpPrintString( " NV contents:\r\n" );
        NvData_Output_NvPageData(&sBlkData);
       
        // proceed to next block
        BLOCK += 1;
    }
}

/*-----------------4/9/2012 2:21PM------------------
 * Print the RAM data for an entry
 * --------------------------------------------------*/
static void NvData_Output_RamData( simeeprom_Block_Info_T * psBlk )
{
    UINT16 nPrintCount, index = 0;  // the number of bytes to print this pass
    UINT8 * pnRamData;              // a byte ram data pointer

    while ( index < (psBlk->data_size - 1) )
    {
        // find the number of bytes to print, which will get shortened at the end of the block
        nPrintCount = ((psBlk->data_size - index) < 16) ? (psBlk->data_size - index) : 16;

        // set the byte pointer to data set
        pnRamData = ((UINT8 *)(psBlk->data_in_RAM) + index);

        // write the start address for each line, and dump out the number of entries
        sprintf(CmdBuffer, " %04Xh:", index );
        NvData_Output_DataLine(pnRamData, nPrintCount);

        // increment the start index by the number of bytes printed this pass and return false until we print them all
        index += nPrintCount;
    }
}

/*-----------------4/10/2012 10:54AM----------------
 * Print a line of data values. the temp string
 * needs to be pre-cleared as this will only cat onto
 * it (so we can preceed with an address or page)
 * --------------------------------------------------*/
static void NvData_Output_DataLine( UINT8 * pnDataBlock, UINT8 byteCount )
{
    char szTemp[8]; // a formatting temp
    UINT8 count;    // the number of bytes

    // write the start address for each line, and dump out the number of entries
    for ( count = 0; count < byteCount; count++ )
    {
        sprintf( szTemp, " %02X", (int)pnDataBlock[count] );
        strcat(CmdBuffer, szTemp);
        if ( count == 7 )
        {
            strcat(CmdBuffer, " ");
        }
    }
    strcat(CmdBuffer, "\r\n");
    LpPrintString(CmdBuffer);
}

/*-----------------4/9/2012 2:21PM------------------
 * Print the RAM data for an entry
 * --------------------------------------------------*/
static void NvData_Output_ConstData( simeeprom_Block_Info_T * psBlk )
{
    UINT16 index = 0;
    UINT8 idx, nPrintCount, pnConstData[16];

    while ( index < (psBlk->data_size - 1) )
    {
        // copy the constant data to the ram array for printing
        for ( idx = 0; idx < 16; idx++ )
        {
            pnConstData[idx] = *((const UINT8 *)(psBlk->default_data) + index + idx);
        }

        // find the number of bytes to print, which will get shortened at the end of the block
        nPrintCount = ((psBlk->data_size - index) < 16) ? (psBlk->data_size - index) : 16;

        // write the start address for each line, and dump out the number of entries
        sprintf(CmdBuffer, " %04Xh:", index );
        NvData_Output_DataLine(pnConstData, nPrintCount);

        // increment the start index by the number of bytes printed this pass and return false until we print them all
        index += nPrintCount;
    }
}

//#if 0 // 08/29/2012 -- Removed to save code space, no need to print entire NVData structure
/*-----------------4/9/2012 3:00PM------------------
 * Print flash/eeprom storage pages associated with
 * a simeeprom structure
 * --------------------------------------------------*/
static void NvData_Output_NvPageData( simeeprom_Block_Info_T * psBlk )
{
    UINT16 nCurrentPage;        // keep track of the current page
    UINT16 nDataOffset;         // offset into the buffer
    UINT16 nPrintBytes;         // number of bytes to print
    UINT8 * pnDestData;         // the current page data (temp)
    BOOL bContinueLoop = TRUE;  // store the logic evaluation to keep looping for more pages

    
    // figure out of the page numbering is in reverse order
    BOOL bIsReversed = (psBlk->last_page < psBlk->first_page) ? TRUE : FALSE;

    // in mirror mode, only need to print the first/last pages, space between is not used. in stripe mode, the range is used
    BOOL bIsStripeMode = simeeprom_IsStripeMode();

    // allocate the memory for a page. if it fails or the size is too big, log an error and leave
    pnDestData = (UINT8 *)NvBuffer;
    if ( pnDestData == NULL )
    {
        //ErrLogAddEntry("NV dump alloc failed", psBlk->page_size, ERRLVL_ERROR);
        return;
    }

    // else good to proceed, seed the page to the start (ok for either mode)
    nCurrentPage = (bIsReversed) ? psBlk->last_page : psBlk->first_page;

    // loop until we find terminating criteria
    while ( bContinueLoop == TRUE )
    {
        // read the data at the currently indexed page for the page size*/
        /*nReadStatus =*/ simeeprom_BSP_ReadData(nCurrentPage, pnDestData, psBlk->page_size);

        // seed the data offset to zero and loop until we print all the data in the page size
        nDataOffset = 0;
        while ( nDataOffset < psBlk->page_size )
        {
            // determine the number of bytes to print, which is based on the bytes per line and what's left in the page size
            nPrintBytes = ((psBlk->page_size - nDataOffset) < 16) ? (psBlk->page_size - nDataOffset) : 16;

            // print out or continue printing the data for this page. after using new-page flag, clear and it may get reset below
            sprintf(CmdBuffer, " %03u, %04Xh:", nCurrentPage, nDataOffset );
            NvData_Output_DataLine(&pnDestData[nDataOffset], nPrintBytes);

            // increment the bytes added to the counter
            nDataOffset += nPrintBytes;
        }

        // figure out if there are more pages and what the next one is
        // for mirror mode (false) we just have to toggle the mode and then we are done
        if ( bIsStripeMode == FALSE )
        {
            // we continue if the current page is the first page to print, which is based on whether it is reversed..
            bContinueLoop = (nCurrentPage == ((bIsReversed) ? psBlk->last_page : psBlk->first_page)) ? TRUE : FALSE;
            // flip the page to print
            nCurrentPage = ((bIsReversed) ? psBlk->first_page : psBlk->last_page);
        }

        // else in stripe mode, we have to print the range of pages. the logic fixes the printout so we always count up
        else
        {
            // this is the inverse logic of the first page
            UINT16 nFinishPage = (bIsReversed) ? psBlk->first_page : psBlk->last_page;

            // advance the page index and when we exceed the finish page, we are done
            nCurrentPage += 1;
            bContinueLoop = (nCurrentPage > nFinishPage) ? FALSE : TRUE;
        }
    }
}

//Ends Warning Area

 #endif 
#endif



/*
 *
 * TSID callbacks section.
 *
 * These callbacks are used by the TSID module to inform of various events, for NV data storage, and other events the module
 *  may need application involvement for
 *
 */


/**
 * \brief   Callback, TSID module, load TSID from NV storage
 * \details This is called at startup/initialization of the TSID module so that the module RAM copy of the TSID can be restored
 *          from a non-volatile data space.\n\n
 *          Since the available NV areas are almost always dependent on a particular target, this is implemented as a callback
 * \return  UINT32
 * \retval  the TSID as a 32b longword
 */
UINT32 CmTsidCbLoadTsidFromNV(void)
{
    UINT32 lTsidValue = appeeprom_ReadTSID();

#ifdef __CNET_DEVICE__
	if((lTsidValue == 0x00000000) || (lTsidValue == 0xFFFFFFFF) )
	{
        // use the standard TSID self-generate behavior to ensure proper generation and reduce redundant code.  this will
        // generate a call back to store to NV after an update tick

        /* WARNING: The TSIDMSK_DIRTY mask will be set when calling this SelfGenerate function to write back the TSID To NVRAM,
        but since this callback is called from CmTsidInitialize() at powerup, the TSIDMSK_DIRTY will get cleared 
        and NO data is saved back to NVRAM. */
        lTsidValue = CmTsidSelfGenerateRandomTsid(TSID_SELFGEN_MAGIC_KEY);
	}
#endif
    return lTsidValue;
}


/**
 * \brief   Callback, TSID module, store TSID to NV storage
 * \details This is called from the TSID module when the TSID changes, so the system non-volatile copy can be updated.\n\n
 *          Since the available NV areas are almost always dependent on a particular target, this is implemented as a callback
 * \return  void
 * \param   lTsidValue A UINT32 with the TSID as a 32b longword
 */
void CmTsidCbStoreTsidToNV( UINT32 lTsidValue )
{
	appeeprom_WriteTSID(lTsidValue);
}


/**
 * \brief   Callback, TSID module, get psuedo-random 32b value
 * \details This is called from the TSID module to self-generate a random TSID value, for Cresnet devices TSID conficts.\n\n
 *          Because more sources of random information are available at the application level than in the modules, this is
 *          implemented as a callback
 * \return  UINT32
 * \retval  "A 32b random value" determined by some suitable process in the application.\n\n
 *          Possible sources might include timer count values, A2D readings, or any data source where the value at any instant
 *          wont be correlated to other devices
 */
UINT32 CmTsidCbGetRand32(void)
{
    return CmTsidRandomize96bitTo32bitPPNSeed( (UINT32 *)0x1FFFF7E8UL ); //Take the 96bit Serial number on MCU and compress down to 32bit randomized format
}


/**
 * \brief   Callback, TSID module, TSID state change event
 * \details This is called from the TSID module to inform the application of a change in TSID states.\n\n
 *          For example, it will be called to notify of an entry or exit of light-and-poll.\n\n
 *          If multiple events occur as a result of a particular TSID action, then this callback will be made with each event
 *          that occured.\n\n
 *          Populate the switch cases that are relevant for the particular target.
 * \return  void
 * \param   cmstaTsidEvent A CmStatusEventErrorCodes enumeration with the TSID event code that occurred
 */
void CmTsidCbOnTsidEvent( CmStatusEventErrorCodes cmstaTsidEvent )
{
	fp_ProcessTsidState( cmstaTsidEvent );

#ifdef __CNET_DEVICE__
	if(cmstaTsidEvent == CMSTA_TSID_VALUE_CHANGE)
	{
		CmLinkBuildTSIDString();
		CmLinkBuildCresnetIdString();
	}
#endif
}



/*
 *
 * Main transport module callbacks section.
 *
 * These callbacks are used by the transport module to request ID information and inform of link/transport status changes
 *
 */

/**
 * \brief   Callback, Transport module, device ID changed
 * \details This is called from the transport module to inform the application that the device ID changed.\n\n
 *          Some applications such as transceiver modules have to know when the ID has changed and report this to the host
 *          application.\n\n
 *          Generally, any call to OnDeviceIdChange will be accompanied by a call to store the device ID, but storage should
 *          not be done in this function.
 * \return  void
 * \param   nNewId A UINT8 with the new ID value that is being used
 */
void CmTransCbOnDeviceIdChange( UINT8 nNewId )
{
    // no need to do anything from here with new resync module
}


/**
 * \brief   Callback, Transport module, load device ID
 * \details This is called from the transport module load the device ID from NV data space.\n\n
 *          Because the NV space available will almost always be based on the target, this is implemented as a callback
 * \return  UINT8
 * \retval  "The Device ID" to use for the device (Cresnet or MNET ID)
 */
UINT8 CmTransCbLoadDeviceIdFromNV(void)
{
    UINT8 nNewDeviceID = appeeprom_ReadDeviceID();
    ErrLogAddEntryBase("[NV] read id     ", nNewDeviceID, ERRLVL_CAUTION,0);
    // todo there used to be some error checking here, should that be done?
    return nNewDeviceID;
}


/**
 * \brief   Callback, Transport module, store device ID
 * \details This is called from the transport module store the device ID to NV data space.\n\n
 *          Because the NV space available will almost always be based on the target, this is implemented as a callback.\n\n
 *          Generally, any call to store Device ID to NV will be accompanied by a call to OnDeviceIdChange, but storage should
 *          only be made by a call to this function
 * \return  void
 * \param   nNewDeviceID A UINT8 with the device ID code to store to NV
 */
void CmTransCbStoreDeviceIdToNV( UINT8 nNewDeviceID )
{
    ErrLogAddEntry("[NV] write id", nNewDeviceID, ERRLVL_CAUTION);
    appeeprom_WriteDeviceID(nNewDeviceID);
}


/**
 * \brief   Callback, Transport module, get device ID string
 * \details This is called from the transport module retrieve the device ID string.\n\n
 *          This shall be only the ASCII section including device name, TSID value and any optional trailing information.\n\n
 *          Some devices have ID strings that change at runtime to show optional information, or they are runtime-dependant
 *          based on PA build.\n\n
 *          No trailing null is necessary because the copy length is returned.
 * \return  UINT8
 * \retval  "Number of bytes" copied into the pointed location
 * \param   nMaxCopyLen A UINT8 with the max number of bytes that the application may copy in
 * \param   pCopyDest A UINT8 * to the location to copy the ID string
 */
UINT8 CmTransCbGetDeviceIdString( UINT8 nMaxCopyLen, UINT8 *pCopyDest )
{    
    // TB needs this to report when the full upgrade (Master and Slave have completed)

    if (slaveUpgradeReady != 0) 
    {
       return  product_BuildDeviceIdString(nMaxCopyLen, pCopyDest );
    }
    else
    {
       return 0;
    }
}

/**
 * \brief   Callback, Transport module, get device ID string extra
 * \details This is called from the transport module retrieve the additional info that needs to be appended to  device ID string.\n\n
 *          This shall be only the ASCII section including device name, TSID value and any optional trailing information.\n\n
 *          Some devices have ID strings that change at runtime to show optional information, or they are runtime-dependant
 *          based on PA build.\n\n
 *          No trailing null is necessary because the copy length is returned.
 * \return  UINT8
 * \retval  "Number of bytes" copied into the pointed location
 * \param   nMaxCopyLen A UINT8 with the max number of bytes that the application may copy in
 * \param   pCopyDest A UINT8 * to the location to copy the ID string
 */

UINT8 CmTransCbGetDeviceIdStringEx( UINT8 nMaxCopyLen, UINT8 *pCopyDest )
{
    #ifdef __MNET_DEVICE__
     #ifdef CSM2
       #if !defined(CSM_QMTDC_163)&&!defined(CSM_DRAPE_MOTOR)
         return product_BuildDeviceIdStringExtended( nMaxCopyLen, pCopyDest);    // Aplicable to 2xx-EX only
       #else 
       return 0;  
       #endif 
     #else
     return 0;
     #endif 
    #else 
    return 0;
    #endif
}

/**
 * \brief   Callback, Transport module, get Tsid string
 * \details This is called from the transport module retrieve the device ID string.\n\n
 *          This shall be only the ASCII section including device name, TSID value and any optional trailing information.\n\n
 *          Some devices have ID strings that change at runtime to show optional information, or they are runtime-dependant
 *          based on PA build.\n\n
 *          No trailing null is necessary because the copy length is returned.
 * \return  UINT8
 * \retval  "Number of bytes" copied into the pointed location
 * \param   nMaxCopyLen A UINT8 with the max number of bytes that the application may copy in
 * \param   pCopyDest A UINT8 * to the location to copy the ID string
 */
UINT8 CmTransCbGetTsidString(UINT8 nMaxCopyLen, UINT8 *pCopyDest)
{        
	return  product_BuildTsidString(nMaxCopyLen, pCopyDest);
}


/**
 * \brief   Callback, Transport module, Acquire state change event
 * \details This is called from the Transport module to inform the application of a change in Acquire status.\n\n
 *          This is called on all events, including entry, exit, and unacquire events
 * \return  void
 * \param   cmstaAcquireEvent A CmStatusEventErrorCodes enumeration with the infiNetEX acquire event code that occurred
 */
void CmTransCbOnAcquireStatusChange( CmStatusEventErrorCodes cmstaAcquireEvent )
{
#ifdef __MNET_DEVICE__ // these status values only apply to infiNet devices
    switch ( cmstaAcquireEvent )
    {
        case CMSTA_INFXACQ_ENTERED:
            console_Print("[CmTransCbOnAcquireStatusChange] Acquire Entered");
            break;
        case CMSTA_INFXACQ_EXIT_SUCCESS:
            console_Print("[CmTransCbOnAcquireStatusChange] Acquire Exit/Success");
            break;
        case CMSTA_INFXACQ_EXIT_FAIL:
            console_Print("[CmTransCbOnAcquireStatusChange] Acquire Exit/Fail");
            break;
        case CMSTA_INFXACQ_EXIT_CANCELLED:
            console_Print("[CmTransCbOnAcquireStatusChange] Acquire Exit/Cancelled");
            break;
        case CMSTA_INFXACQ_UNACQUIRE:
            console_Print("[CmTransCbOnAcquireStatusChange] UnAcquire");
            break;

        // Auto-acquire was entered at boot; we need to notify the fp handler to show the aquire state
        case CMSTA_INFXACQ_AUTOACQ_ENTERED:
            #ifdef __MNET_DEVICE__
                console_Print("[CmTransCbOnAcquireStatusChange] AutoAcquire entry");
                event_AcquireButtonIsPressed(NULL, NULL, EVENT_ACQUIRE_AUTOSTART, NULL);
            #endif
            break;
    }
#endif
}


/**
 * \brief   Callback, Transport module, link state change event
 * \details This is called from the Transport module to inform the application of a change in link status. The range of link
 *          status values is dependant on medium.\n\n
 *          RF devices can be linked to their GW but not to the control system (poll state bad).\n\n
 *          Cresnet devices are either connected or not connected because their only means of detecting a connection is being
 *          polled, which also means they're in the program.
 * \note    Cresnet devices and RF devices that don't care about explicit poll state (i.e. a repeater controlling its link LED)
 *          should treat both LINKGOOD cases as 'on' and LINKBAD as 'off'
 * \return  void
 * \param   cmstaLinkChangeEvent A CmStatusEventErrorCodes enumeration with the link change event code that occurred
 */
void CmTransCbOnLinkStatusChange( CmStatusEventErrorCodes cmstaLinkChangeEvent )
{
}


/**
 * \brief   Callback, Transport module, event message exchange
 * \details This is called from the Transport module to inform the application of an inter-module message.\n\n
 *          This system is primarily for future compatibility and allows modules to communicate to each other without having to
 *          know about who the other modules may be at build time.\n\n
 *          This is also advantageous for events to which multiple modules need to be notifed
 * \note    In order to receive messages with this callback, the application or module needs to register the callback with the
 *          messaging system in the transport module
 * \return  void
 * \param   cmstaMessage A CmStatusEventErrorCodes enumeration with an event code
 * \param   nCallerID A UINT8 with a caller id code to identify who the message came from.  This can be used to filter
 *          messages from modules that the callback isn't interested in
 * \param   nParam16 A UINT16 with a 16b parameter whose usage depends on the message
 */
void CmAppCbOnIncomingModuleMessage( CmStatusEventErrorCodes cmstaMessage, UINT8 nCallerID, UINT16 nParam16 )
{
}


/**
 * \brief   Callback, Transport module, direct Cresnet RX message
 * \details This is an optional, direct RX callback.\n\n
 *          This is only implemented if we are not using the Receive Parsing module.\n\n
 *          If we are using it, we get a collection of decoded callbacks plus a catch-all rx packet callback instead.
 * \return  CmStatusEventErrorCodes
 * \retval  CMSTA_SUCCESS for now, may be used for other success/error codes later
 * \param   nPktLen A UINT8 with the length of the packet.  For safety, don't assume byte 1 has a valid length
 * \param   pRxDataSrc A UINT8 * to the RX packet location.\n\n
 * \note    If saving data for later use, copy the packet inside this call, do not save pointer value
 */
#if ( CM_USE_MODULE_RX_PARSING == 0 )
CmStatusEventErrorCodes CmTransCbOnCresnetRxPacket( UINT8 nPktLen, UINT8 *pRxDataSrc )
{
    return CMSTA_SUCCESS; // return code not implemented yet
}
#endif  // CM_USE_MODULE_RX_PARSING


/**
 * \brief   Callback, Transport module, direct Cresnet TX message
 * \details This is an optional, direct TX callback.\n\n
 *          This is only implemented if we are not using the Transmit Buffering module.\n\n
 *          If we are using it, we get a collection of calls to load particular events or signal types.  To use this callback,
 *          copy a complete, properly over-the-wire-formatted Cresnet packet to the pointed location and return the number of
 *          bytes copied
 * \return  UINT8
 * \retval  "The number of bytes" copied into the transmit buffer.\n\n
 *          If we don't have a packet to transmit right now, simply return 0 and don't write anything to the pointed location
 * \param   nMaxTxBytes A UINT8 with the maximum length of TX packet that can be written to the pointed location
 * \param   pTxDataDest A UINT8 * to the location to write the Cresset packet
 */
#if ( CM_USE_MODULE_TX_BUFFERING == 0 )
UINT8 CmTransCbGetCresnetTxPacket( UINT8 nMaxTxBytes, UINT8 *pTxDataDest )
{

    return 0;
}
#endif  // CM_USE_MODULE_TX_BUFFERING


/**
 * \brief   Callback, Transport module, transport state change event
 * \details This is called from the Transport module to inform the application about changes in the state of the transport
 *          system.\n\n
 *          Because the transport system hides the transport means, only generalized events are reported.\n\n
 *          More detailed information can be retrieved using a call to the get transport state.
 * \note    Most applications won't need to use this callback.\n\n
 *          This callback is a means for retrieving more advanced information about the transport operation that may only be
 *          useful for specific purposes
 * \return  void
 * \param   cmstaTransportEvent A CmStatusEventErrorCodes enumeration with the transport status change/event that occurred
 * \param   nExtendedData16 A UINT16 parameter that may contain extended data about the event
 */
void CmTransCbOnTransportStatusChange( CmStatusEventErrorCodes cmstaTransportEvent, UINT16 nExtendedData16 )
{
}


/**
 * \brief   Callback, Cnet File Transfer Start (Upgrade UPG) Event
 * \details Gets packet from transport to be processed by Cnet FW Upgrade Module
 * \return  void
 */
void CmRxpCbOnFileTransferMisc( UINT8 *pRxDataSrc )
{
#ifdef __CNET_DEVICE__ // this upload mode only applies to Cresnet
	CmLinkProcessUpgradeCommand( pRxDataSrc );
#endif
}


/**
 * \brief   Callback, Cnet SRecord Packet Event
 * \details Gets packet from transport to be processed by Cnet FW Upgrade Module
 * \return  void
 */
void CmRxpCbOnSrecord( UINT8 *pRxDataSrc )
{
#ifdef __CNET_DEVICE__ // this upload mode only applies to Cresnet
	cnupg_ProcessOneRecord( pRxDataSrc );
#endif
}


/**
 * \brief   Callback, Transport full parse module, check for mutlicast ID valid
 * \details This will be called when the full parse begins decoding a packet embedded in a verified (address/CRC) type 45h
 *          mutlicast packet.  The app can decide whether to allow futher parsing of the packet by returing true/false here.
 *          While the app may use the second two params to snoop the data coming in (and make a final decision early), this
 *          may not be very efficient.  Instead, it may wish to use the CmTrnsFpIsMcBc45hContext and CmTrnsFpGetMcBc45hID
 *          interface from within the decoded RX event callbacks to select how data is processed.  This callback can be useful
 *          for broad filtering; i.e. if a device knows a particular MC address is not valid at all, it can reject the packet
 *          here to save time in later parsing.
 * \return  BOOL
 * \retval  TRUE if the ID and packet conditions can be acted upon as a multicast
 * \retval  FALSE otherwise
 * \param   MCID A UINT8 with the multicast ID of the packet
 * \param   length A UINT8 with the number of bytes in the current encapsulated packet
 * \param   pnData A UINT8 * to the packet data
 */
#ifdef __BCMC45h_SUPPORT_ENABLED
BOOL CmTransCbIsMcIdValid( UINT8 MCID, UINT8 length, UINT8 * pnData )
{
    //return (MCID == 0xFF) ? TRUE : FALSE;  // in general, if a device supports 45h MC, it should respond to FFh

    if ( MCID == 0xFF )                      // in general, if a device supports 45h MC, it should respond to FFh
    {
        return TRUE; 
    }

    if ( MCID > 0 )
    {
        //Check if any of the GroupIDs equals to the MCID
        for (int i =0; i < 6; i++)
        {
            if ( MCID == sSfyJoins.sSlot20.nGroupID[i])
            {
                return TRUE;
            }
        }
    }

    return FALSE;

}
#endif





/*
 *
 * Receive Parsing callbacks section.
 *
 * These callbacks are used by the receive parsing module to report incoming signals and other common events
 *
 */


/**
 * \brief   Callback, RX Parsing module, StartOfProgram or SystemRestart event
 * \details An all-clear is the broadcast (0xFF) version
 * \return  void
 */
void CmRxpCbOnAllClear(void)
{
}


/**
 * \brief   Callback, RX Parsing module, Start-of-update event
 * \details A start-of-update is the unicast (id'd) version
 * \return  void
 */
void CmRxpCbOnStartOfUpdate(void)
{
}


/**
 * \brief   Callback, RX Parsing module, end-of-update event
 * \details An end of update is received after the data updating (from an update request) is completed
 * \return  void
 */
void CmRxpCbOnEndOfUpdate(void)
{  
}


/**
 * \brief   Callback, RX Parsing module, end-of-init event
 * \details An end of init is received after the data updating (from a system/program restart) is completed
 * \return  void
 */
void CmRxpCbOnEndOfInitialization(void)
{
}


/**
 * \brief   Callback, RX Parsing module, digital event
 * \return  void
 * \param   nSlot A UINT8 with the slot (0 for main slot)
 * \param   nJoin16 A UINT16 with the join of the input
 * \param   bIsPress A BOOL that is true if the input was a press
 */
void CmRxpCbOnDigital( UINT8 nSlot, UINT16 nJoin16, BOOL bIsPress )
{
    ShdDbgCntrOnDigital(nSlot, nJoin16, bIsPress);              // may need to log this for debugging purposes
    event_CnetRxDigitalInputHandler(nSlot, nJoin16, bIsPress);  // handle digital data
}


/**
 * \brief   Callback, RX Parsing module, analog event
 * \details All analogs, regardless of over-the-wire format, will report with 16b channels and 16b values.\n\n
 *          If a value is considered signed by the application, it will have to be cast here.\n\n
 *          All analog callbacks are 0-based channel
 * \return  void
 * \param   nSlot A UINT8 with the slot (0 for main slot)
 * \param   nJoin16 A UINT16 with the join of the input
 * \param   nValue16 A UINT16 with the value of the signal
 */
void CmRxpCbOnAnalog( UINT8 nSlot, UINT16 nJoin16, UINT16 nValue16 )
{
    //#warning Print debug on Analog join
    //console_Print("[CmRxpCbOnAnalog] %xh, %u/%u/%u\r\n", CmSyncIsUpdateInProgress(), nSlot, nJoin16, nValue16 );

    // for now, do not process position commands received during the update
    
    if ( (CmSyncIsUpdateInProgress() == TRUE) && (nSlot == SLOT_01) && (nJoin16 == JOIN_02) )
    {
       // Not allowing any pos join during UPREQ //if ( nValue16 == 0) 
       {
           ErrLogAddEntryBase("Pos denied: UpReq", csm_ctrl_getDir(), ERRLVL_CAUTION, getPosPercent());
      
           //Should also log this state: CmSyncGetSyncState()
           return;
       }
    }

    #ifdef CSM_QMTDC_163

    if ( (CmSyncIsUpdateInProgress() == TRUE) && (nSlot == SLOT_01) && (nJoin16 == JOIN_04) )
    {
       // Not allowing any pos join during UPREQ // if ( nValue16 == 0) 
       {
           ErrLogAddEntryBase("Tilt denied: UpReq", csm_ctrl_getDir(), ERRLVL_CAUTION, getPosPercent());

           //Should also log this state: CmSyncGetSyncState()
           return;
       }
    }
    #endif


    ShdDbgCntrOnAnalog(nSlot, nJoin16, nValue16);               // may need to log this for debugging purposes
    event_CnetRxAnalogInputHandler(nSlot, nJoin16, nValue16);   // handle the analog
}


/**
 * \brief   Callback, RX Parsing module, serial event
 * \details Called on string (serial) RX
 * \note    Copy serial data inside this call, do not save pointer value for later use
 * \return  void
 * \param   nSlot A UINT8 with the slot (0 for main slot)
 * \param   nJoin16 A UINT16 with the join of the input
 * \param   nStrLen A UINT8 with the length of the string
 * \param   pSource A UINT8 * to the location of the string data
 */
void CmRxpCbOnSerial( UINT8 nSlot, UINT16 nJoin16, UINT8 nStrLen, UINT8 *pSource )
{
}


/**
 * \brief   Callback, RX Parsing module, Lighting RCB event
 * \return  void
 * \param   nSlot A UINT8 with the slot (0 for main slot)
 * \param   nChannel A UINT16 with the lighting channel
 * \param   nTermLevel A UINT16 with the terminating level for the ramp
 * \param   lRampTime10ms A UINT32 with the ramp time in native 10ms ticks
 */
void CmRxpCbOnRCB( UINT8 nSlot, UINT16 nChannel, UINT16 nTermLevel, UINT32 lRampTime10ms )
{
    //console_Print("[CmRxpCbOnRCB] %xh, %u/%u/%u/%lu\r\n", CmSyncIsUpdateInProgress(), nSlot, nChannel, nTermLevel, lRampTime10ms );

    // for now, do not process position commands received during the update
    if ( (CmSyncIsUpdateInProgress() == TRUE) && (nSlot == SLOT_01) && (nChannel == JOIN_02) )
    {
        return;
    }

    ShdDbgCntrOnRCB(nSlot, nChannel, nTermLevel, lRampTime10ms);                // may need to log this for debugging purposes
    event_CnetRxRCBInputHandler( nSlot, nChannel, nTermLevel, lRampTime10ms );  // handle the RCB data
}


/**
 * \brief   Callback, RX Parsing module, general paramter event
 * \return  void
 * \param   nSlot A UINT8 with the slot (0 for main slot)
 * \param   nJoin A UINT16 with the join/index of the param
 * \param   lParamValue A UINT32 with the value of the paramter as 32b
 */
void CmRxpCbOnParameter( UINT8 nSlot, UINT16 nJoin, UINT32 lParamValue )
{
    //#warning debug msg
    //console_Print("param: %d  %d",nSlot, nJoin);

    ShdDbgCntrOnParam(nSlot, nJoin, lParamValue);                   // may need to log this for debugging purposes
                                                                    
    event_CnetRxParameter32InputHandler(nSlot, nJoin, lParamValue); // handle the parameter data
}


/**
 * \brief   Callback, RX Parsing module, Paramter Dynamic/Status event
 * \return  void
 * \param   void 
 */

void CmRxpCbOnStartOfParameter( void )
{
}

void CmRxpCbOnEndOfParameter( void )
{
}

/**
 * \brief   Callback, RX Parsing module, general paramter event
 * \return  void
 * \param   nSlot A UINT8 with the slot (0 for main slot)
 * \param   nJoin A UINT16 with the join/index of the param
 * \param   nStringLen A UINT8 with the size of the incoming string
 * \param   pSource A pointer to the source string
 */
void CmRxpCbOnStringParameter( UINT8 nSlot, UINT16 nJoin, UINT8 nStringLen, UINT8 *pSource )
{
}


void CmRxpCbOnVersiport( UINT8 nStringLen, UINT8 *pSource )
{
}


/*-----------------7/18/2012 5:06PM-----------------
 * passto commands I think
 * --------------------------------------------------*/
void CmRxpCbOnMultiSerial( UINT8 nSlot, UINT8 nFlag, UINT8 nChannel, UINT8 nSubIndex, UINT8 nPktLen, UINT8 * pSource)
{
// we only receive passto commands this way on a Cresnet device (for now anyway)
#ifdef __CNET_DEVICE__
	if ( nSlot == 0 )
	{
		console_ProcessType12( nFlag, nSubIndex, nPktLen, pSource );
	}
#endif
}


/**
 * \brief   Callback, RX Parsing module, RCON string input
 * \return  void
 * \note    Copy the string inside this call, do not save pointer value for later use
 * \param   nStringLen A UINT8 with the length of the command string
 * \param   pSource A UINT8 * to the command string
 */
void CmRxpCbOnRCON( UINT8 nStringLen, UINT8 *pSource )
{
// we only receive RCON commands this way on a Cresnet device
#ifdef __CNET_DEVICE__
	console_ProcessRconCommand(nStringLen, pSource);
#endif
}


/**
 * \brief   Callback, RX Parsing module, other RX packet
 * \details This is the catch-all callback that passes the entire RX packet to the application if it cannot be decoded in the
 *          RX parsing module.\n\n
 *          The application can do its own limited parsing here for the things that are not captured by the other callbacks
 * \note    Copy the packet inside this call, do not save pointer value for later use
 * \return  void
 * \param   nPktLen A UINT8 with the length of the packet
 * \param   pRxPkt A UINT8 * to the packet
 */
void CmRxpCbOnRxOther( UINT8 nPktLen, UINT8 *pRxPkt )
{
}


/**
 * \author  Jeffrey Ludlow
 * \brief   callback for the application timer events
 * \date    02/24/10
 * \return  CmevtStatus
 * \retval  "a CmevtStatus value"    if handled
 * \retval  CMEVT_STATUS_UNKNOWN     if input handle unknown
 * \param   nHANDLE                 UINT8 with the handle
 * \param   tmTimerMode             CmevtTimerMode with the timer mode for the event that came due
 */

CmevtStatus CmevtAppTmrEventCallback( UINT8 nHANDLE, CmevtTimerMode tmTimerMode )
{
    CmevtStatus lpsReturnStatus = CMEVT_STATUS_UNKNOWN;

    return lpsReturnStatus;

}


/*-----------------6/5/2012 4:48PM------------------
 * New update handler callback
 * Called on start of updating period
 * --------------------------------------------------*/
void CmSyncCbOnUpdateStart(void)
{
    event_ProcessClearAll();        // clear any output states if needed @@@@
    sys_SimplProgramDisConnected(); // Indicate now that program is disconnected, wait for new update request
   
    //console_Print("[CmSyncCbOnUpdateStart] %xh\r\n", CmSyncIsUpdateInProgress() );

    //Mechanism to clear a state where the End of Update Request never came in
    ticktimer_Clear(TIMERS_WAITING_END_OF_UPDATE_REQUEST);
    ErrLogAddEntryBase("Start UpdtReq", 0, ERRLVL_CAUTION, 0);

}


/*-----------------6/5/2012 4:49PM------------------
 * New update handler callback
 * Called on end of updating period
 * --------------------------------------------------*/
void CmSyncCbOnUpdateComplete(void)
{
    g_System.m_ProcessEvent.SymbolParameters = TRUE; //set flag to process the params stored later in main loop area
    sys_SimplProgramConnected(); //indicate to application that simpl program is now connected to device
    event_ProcessUpdateRequest();    
    ErrLogAddEntryBase("@Update Complete", 0, ERRLVL_NOTICE, 0); //@@@@
    //console_Print("[CmSyncCbOnUpdateComplete] %xh\r\n", CmSyncIsUpdateInProgress() );
}


/*-----------------6/5/2012 4:49PM------------------
 * New update handler callback
 * Called on any detectable error in the updating
 * process.
 * --------------------------------------------------*/
void CmSyncCbOnUpdateError(CmStatusEventErrorCodes status)
{
}

/**
 * \brief   Returns a descriptor for the selected application event timer handle
 * \date    03/03/2011
 * \return  PGM_P constant string pointer
 * \retval  A pgm area pointer to the descriptor string
 */
PGM_P CmevtCbGetAppTimerDescriptor( UINT8 nEventHandle )
{
    return "{null}"; // failsafe return value
}



// used for somfy NV data read (from the motor board)
extern void sfy_Nvd_StartReadSequence(void);


/*-----------------5/17/2012 5:53PM-----------------
 * Handle the debug mask set/read command
 * --------------------------------------------------*/
static void OnCmdMotorDebugMask(void)
{
    // Given the fact that the application NV storage shadows NV settings, implement this set/get storage interface there
    // instead of storing the value locally
    extern UINT8 ShdDbgGetDebugMask(void);
    extern void ShdDbgSetDebugMask( UINT8 nNewMask );

    // set the value only if we have an argument, otherwise just return its value
    if ( console_GetArgCount() >= 1 )
    {
        //UINT32 newMask = strtoul( console_GetArgPtr(0), NULL, 16 );
        UINT32 newMask = console_GetNextParamAsciiHex();
        ShdDbgSetDebugMask((UINT8)newMask);
    }
    #ifdef __CNET_DEVICE__
        console_Print( "Shade debug mask %02Xh\r\n", ShdDbgGetDebugMask() );
    #else
        console_Print( "Shade debug mask %xh\r\n", ShdDbgGetDebugMask() );
    #endif
}


/**
 * \brief   Dump the local NV motor settings
 * \details Implemented from the application size as some of the parameters are not standard across all shade types
 * \return  void
 */
void csmRtDbgConsoleOpData(void)
{
    // this string table describes the op modes
    // must align with other enum ShadeOpState
    static const char * pszcOpModeDescriptors[] = 
    {
        "No Limits",    // SS_NO_LIMITS_SET
        "Normal",       // SS_NORMAL
        "Set-Upper",    // SS_SET_UPPER_LIMIT
        "Set-Lower",    // SS_SET_LOWER_LIMIT
        "Set-Tilt",      // SS_SET_LOWER_LIMIT
        "Enter-Limits"
    };
    SHD_CMD_UPDATE sMotorData;
    UINT32 percent;
    UINT16 level16b;

    // fetch the current motor operating data    
    ShdDbgGetOpData(&sMotorData);

    // find the shade position as an analog level
    level16b = ShdLimGetLevelFromPosition(EEPROM_Sys.sys_nUpperLimit, EEPROM_Sys.sys_nLowerLimit, ctrl_GetNowPosition());//sMotorData.position);
    percent = ((100 * (UINT32)level16b) / 65535);

    // some title stuff
    console_PutString( "\r\n");
    console_PutString( "Motor settings and status:\r\n");


    //A very simplified print out for the EM3588
#if defined(CORTEXM3_EM3588) 
     //Position
     console_Print(
            " Current pos: %u (%u%%)\r\n"
            " Upper limit: %u\r\n"
            " Lower limit: %u\r\n"
            " Tilt  offset: %d\r\n"
            " Tilt  limit:  %u\r\n",
            sMotorData.position, percent,
            EEPROM_Sys.sys_nUpperLimit,
            EEPROM_Sys.sys_nLowerLimit,
            EEPROM_Sys.sys_silhouetteLimit,
            getFullTiltTicks()+EEPROM_Sys.sys_nLowerLimit);

     //Speed 
     console_Print(
            " Command speed:    %d [RPM]\r\n"
            " Cresnet speed:    %u [RPM]\r\n"
            " Measured speed:   %d [RPM]\r\n"
            " Jog time:         %u [ms]\r\n"
            " Jog kick time:    %u [ms]\r\n"
            " Raise direction:  %s\r\n",
            (sMotorData.speedRpmX10 / 10),
            EEPROM_Param.prm_nMotorSpeedRpm,
            ShdPctrlGetAvgSpeed(),
            EEPROM_Param.prm_nJogTime,
            EEPROM_Param.prm_nKickTime,
         #ifdef CSM_QMTDC_163
            (EEPROM_Sys.sys_nRaiseDir == 0x00) ? "CW" : "CCW"
         #else
            (EEPROM_Sys.sys_nRaiseDir == 0x00) ? "CCW" : "CW" 
         #endif  

        );

     //Limits
      console_Print(
            " Upper limit set:  %c\r\n"
            " Lower limit set:  %c\r\n"
            " Tilt limit set:  %c\r\n"
            " Limit set t/o:    %u\r\n"
            " Lim orientation:  %s\r\n"
            " Operation mode:   %s\r\n"
            " Error flags mask: %u\r\n",
            ctrl_IsLimitValidUpper() ? 'Y' : 'N',
            ctrl_IsLimitValidLower() ? 'Y' : 'N',
            ctrl_IsLimitTiltAdjusted() ? 'Y' : 'N',
            ctrl_GetLimitStoreTimeout(),
            ShdLimGetOrnttnString( ctrl_GetLimitsOrientation() ),
            pszcOpModeDescriptors[ctrl_GetShdOpState()],
            ctrl_GetSysErrorMask()
        );

      //version info
      console_Print( " Motor revision:   %u.%u.%u\r\n",
            ((int)*(ctrl_GetMotorFirmwareRev() + 2) & 0x0FF),
            ((int)*(ctrl_GetMotorFirmwareRev() + 1) & 0x0FF),
            ((int)*(ctrl_GetMotorFirmwareRev() + 0) & 0x0FF)
        );

      console_Print( " Motor app ID:     %u\r\n", ctrl_GetMotorFirmwareAppID() );
      console_Print( " Loopback active:  %c\r\n", diag_IsActive() ? 'Y' : 'N' );

     return;

#else    // defined(CORTEXM3_EM3588)
    // position information
    #ifdef __CNET_DEVICE__
        console_Print(" Current position: %08Xh/%lu (%u/%u%%)\r\n", ctrl_GetNowPosition(), sMotorData.position, level16b, percent);
        console_Print(" Upper limit:      %08Xh/%lu\r\n", EEPROM_Sys.sys_nUpperLimit, EEPROM_Sys.sys_nUpperLimit);
        console_Print(" Lower limit:      %08Xh/%lu\r\n", EEPROM_Sys.sys_nLowerLimit, EEPROM_Sys.sys_nLowerLimit);
        console_Print(" Tilt  offset:     %08Xh/%d\r\n", EEPROM_Sys.sys_silhouetteLimit,  EEPROM_Sys.sys_silhouetteLimit);
//        console_Print(" Tilt limit:       %08Xh/%lu\r\n", getFullTiltTicks()+EEPROM_Sys.sys_nLowerLimit,  getFullTiltTicks()+EEPROM_Sys.sys_nLowerLimit);

    #else // __CNET_DEVICE_
    {
        console_Print(
            " Current position: %4xh/%U (%u/%u%%)\r\n"
            " Upper limit:      %4xh/%U\r\n"
            " Lower limit:      %4xh/%U\r\n",
//            " Tilt limit:       %4xh/%d\r\n",
            ctrl_GetNowPosition(), sMotorData.position, level16b, percent,
            EEPROM_Sys.sys_nUpperLimit, EEPROM_Sys.sys_nUpperLimit,
            EEPROM_Sys.sys_nLowerLimit, EEPROM_Sys.sys_nLowerLimit
//            EEPROM_Sys.sys_silhouetteLimit,EEPROM_Sys.sys_silhouetteLimit
        );
    }
    #endif // __CNET_DEVICE__
    // speed and movement data
    #ifdef __CNET_DEVICE__

        console_Print( " Command speed:    %d [RPM]\r\n", (sMotorData.speedRpmX10 / 10) );
        console_Print( " Cresnet speed:    %u [RPM]\r\n", EEPROM_Param.prm_nMotorSpeedRpm );
        console_Print( " Measured speed:   %d [RPM]\r\n", ShdPctrlGetAvgSpeed() );

        console_Print( " Jog time:         %U [ms]\r\n", EEPROM_Param.prm_nJogTime );
        console_Print( " Jog kick time:    %u [ms]\r\n", EEPROM_Param.prm_nKickTime );
       #ifdef CSM_QMTDC_163
        console_Print( " Raise direction:  %s\r\n", (EEPROM_Sys.sys_nRaiseDir == 0x00) ? "CW" : "CCW" );
       #else
        console_Print( " Raise direction:  %s\r\n", (EEPROM_Sys.sys_nRaiseDir == 0x00) ? "CCW" : "CW" );
       #endif

    #else   // __CNET_DEVICE__
    {
        console_Print(
            " Command speed:    %d [RPM]\r\n"
            " Cresnet speed:    %u [RPM]\r\n"
            " Measured speed:   %d [RPM]\r\n"
            " Jog time:         %U [ms]\r\n"
            " Jog kick time:    %u [ms]\r\n"
            " Raise direction:  %s\r\n",
            (sMotorData.speedRpmX10 / 10),
            EEPROM_Param.prm_nMotorSpeedRpm,
            ShdPctrlGetAvgSpeed(),
            EEPROM_Param.prm_nJogTime,
            EEPROM_Param.prm_nKickTime,
         #ifdef CSM_QMTDC_163
            (EEPROM_Sys.sys_nRaiseDir == 0x00) ? "CW" : "CCW"
         #else
            (EEPROM_Sys.sys_nRaiseDir == 0x00) ? "CCW" : "CW" 
         #endif  

        );
    }
    #endif  // __CNET_DEVICE__

    // limits configuration and error/state
    #ifdef __CNET_DEVICE__
        console_Print( " Upper limit set:  %c\r\n", ctrl_IsLimitValidUpper() ? 'Y' : 'N' );
        console_Print( " Lower limit set:  %c\r\n", ctrl_IsLimitValidLower() ? 'Y' : 'N' );
        console_Print( " Tilt limit set:   %c\r\n", ctrl_IsLimitTiltAdjusted() ? 'Y' : 'N' );        
        console_Print( " Limit set t/o:    %u\r\n", ctrl_GetLimitStoreTimeout() );
        console_Print( " Lim orientation:  %s\r\n", ShdLimGetOrnttnString( ctrl_GetLimitsOrientation() ) );
        console_Print( " Operation mode:   %s\r\n", pszcOpModeDescriptors[ctrl_GetShdOpState()] );
        console_Print( " Error flags mask: %04Xh\r\n", ctrl_GetSysErrorMask() );
    #else // __CNET_DEVICE__
    {
        console_Print(
            " Upper limit set:  %c\r\n"
            " Lower limit set:  %c\r\n"
            " Tilt limit set:  %c\r\n"
            " Limit set t/o:    %u\r\n"
            " Lim orientation:  %s\r\n"
            " Operation mode:   %s\r\n"
            " Error flags mask: %2xh\r\n",
            ctrl_IsLimitValidUpper() ? 'Y' : 'N',
            ctrl_IsLimitValidLower() ? 'Y' : 'N',
            ctrl_IsLimitTiltAdjusted() ? 'Y' : 'N',
           ctrl_GetLimitStoreTimeout(),
            ShdLimGetOrnttnString( ctrl_GetLimitsOrientation() ),
            pszcOpModeDescriptors[ctrl_GetShdOpState()],
            ctrl_GetSysErrorMask()
        );
    }
    #endif // __CNET_DEVICE__
    // versioning information
    #ifdef __CNET_DEVICE__
        console_Print( " Motor version:    %u\r\n", (int)ctrl_GetMotorFirmwareVersion() );

        #ifdef CSM2
            console_Print( " Motor revision:   %02X.%02X.%02Xh\r\n",
                ((int)*(ctrl_GetMotorFirmwareRev() + 2) & 0x0FF),
                ((int)*(ctrl_GetMotorFirmwareRev() + 1) & 0x0FF),
                ((int)*(ctrl_GetMotorFirmwareRev() + 0) & 0x0FF)
            );
            #else 
            {
            console_Print( " Motor revision:   %02X.%02X.%02Xh\r\n",
                ((int)*(ctrl_GetMotorFirmwareRev() + 0) & 0x0FF),
                ((int)*(ctrl_GetMotorFirmwareRev() + 1) & 0x0FF),
                ((int)*(ctrl_GetMotorFirmwareRev() + 2) & 0x0FF)
            );
            }
        #endif
            console_Print( " Motor app ID:     %02Xh\r\n", ctrl_GetMotorFirmwareAppID() );
            console_Print( " Loopback active:  %c\r\n", diag_IsActive() ? 'Y' : 'N' );
        #else 
            {
          console_Print(
            " Motor version:    %u\r\n"
            " Motor revision:   %x.%x.%xh\r\n"
            " Motor app ID:     %xh\r\n"
            " Loopback active:  %c\r\n",
             (int)ctrl_GetMotorFirmwareVersion(),
            ((int)*(ctrl_GetMotorFirmwareRev() + 0) & 0x0FF),
            ((int)*(ctrl_GetMotorFirmwareRev() + 1) & 0x0FF),
            ((int)*(ctrl_GetMotorFirmwareRev() + 2) & 0x0FF),
            ctrl_GetMotorFirmwareAppID(),
            diag_IsActive() ? 'Y' : 'N'
        );
    #endif

#endif
    //console_Print(" Flip LED colors: %c\r\n", (EEPROM_Param.prm_bFlipLEDs != 0) ? 'Y' : 'N' );
}


/*-----------------11/5/2012 3:57PM-----------------
 * Return a mask for the selected LED channel (index)
 * --------------------------------------------------*/
static UINT32 csmGetLedMask( UINT8 ledIndex )
{
    return (UINT32)(0x1 << fp_GetLEDChannel(ledIndex));
}

void csmMassEraseEEPROM(void)
{
    simeeprom_BSP_MassErase();
    return;
}

/*-----------------5/31/2012 3:44PM-----------------
 * Show the user interface state for debugging
 * --------------------------------------------------*/
static void csmShowUiState(void)
{
    #include "csm_fp.h"
    char const * pszButtonStates[] = {"Released", "Pressed" };
    char const * pszLedColors[] =
    {
        "(off)",    // 0b000 = off
        "Green",    // 0b001 = rbG
        "Blue",     // 0b010 = rBg
        "Cyan",     // 0b011 = rBG
        "Red",      // 0b100 = Rbg
        "Orange",   // 0b101 = RbG
        "Pink",     // 0b110 = RBg
        "White"     // 0b111 = white
    };

    // build up the led into a bitmask for indexing
    UINT8 index = led_IsDriven(csmGetLedMask(LED_GRN)) ? 0x01 : 0x00;
    if ( led_IsDriven( csmGetLedMask(LED_BLU)) )
    {
        index |= 0x02;
    }
    if ( led_IsDriven(csmGetLedMask(LED_RED)) )
    {
        index |= 0x04;
    }


    #ifdef __CNET_DEVICE__
        console_Print( "User interface state:\r\n" );
        console_Print( " Button: Open:  %p\r\n", pszButtonStates[ btn_IsButtonPressActive(BTN_OPEN) ] );
        console_Print( " Button: Setup: %p\r\n", pszButtonStates[ btn_IsButtonPressActive(BTN_SETUP) ] );
        console_Print( " Button: Close: %p\r\n", pszButtonStates[ btn_IsButtonPressActive(BTN_CLOSE) ] );
        console_Print( " FP state:      %u\r\n", fp_GetState() );
        console_Print( " LED state:     %p\r\n", pszLedColors[index] );
    #else
        console_Print(
            "User interface state:\r\n"
            " Button: Open:  %p\r\n"
            " Button: Setup: %p\r\n"
            " Button: Close: %p\r\n"
            " FP state:      %u\r\n"
            " LED state:     %p\r\n",
            pszButtonStates[ btn_IsButtonPressActive(BTN_OPEN) ],
            pszButtonStates[ btn_IsButtonPressActive(BTN_SETUP) ],
            pszButtonStates[ btn_IsButtonPressActive(BTN_CLOSE) ],
            fp_GetState(),
            pszLedColors[index]
        );
    #endif
}


/*-----------------6/6/2012 6:15PM------------------
 * Set the jog time in 10ms ticks
 * --------------------------------------------------*/
static void csmSetJogTimeTicks(void)
{
    // set the value only if we have an argument, otherwise just display its value
    if ( console_GetArgCount() >= 1 )
    {
        EEPROM_Param.prm_nJogTime = console_GetNextParamDecimal();
        ctrl_FixupOpParamValues(FALSE);  // false for clamp mode
        ctrl_SetEepromWriteFlag(TRUE);
    }
    console_Print( "Shade jog time ticks %u (%u [ms])\r\n", EEPROM_Param.prm_nJogTime, (10*EEPROM_Param.prm_nJogTime) );
}


/*-----------------6/21/2012 6:22PM-----------------
 * Used for changing the direction reversal kick time
 * to take up the gear backlash.  Otherwise the first
 * few jogs in a direction change get lost
 * --------------------------------------------------*/
static void csmSetJogDirRevKickTime(void)
{
    // set the value only if we have an argument, otherwise just display its value
    if ( console_GetArgCount() >= 1 )
    {
        EEPROM_Param.prm_nKickTime = console_GetNextParamDecimal();
        ctrl_FixupOpParamValues(FALSE);  // false for clamp mode
        ctrl_SetEepromWriteFlag(TRUE);
    }
    console_Print( "Shade jog kick time ticks %u (%u [ms])\r\n", EEPROM_Param.prm_nKickTime, (10*EEPROM_Param.prm_nKickTime) );
}


/*-----------------10/17/2012 11:36AM---------------
 * Display data about the motor overcurrent protection
 * system
 * --------------------------------------------------*/
static void csmDisplayMotorOCPD(void)
{
    console_Print( "Shade overcurrent protection data:\r\n");
    console_Print( " OC detections: %U\r\n", EEPROM_Param.lOvrCurrFlags );
    console_Print( " Rev events:    %U\r\n", EEPROM_Param.lOvrCurrRevEvents );
    console_Print( " Seq thresh:    %u\r\n", EEPROM_Param.nOvrCurrSmplThresh );
    console_Print( " Reverse time:  %u (%u [ms])\r\n",
        EEPROM_Param.nOvrCurrRevTimeMS10,
        (10 * EEPROM_Param.nOvrCurrRevTimeMS10)
    );
    console_Print( " Invrsn flags:  %U\r\n", CsmScGetInvFlagCount() );
}


/*-----------------10/17/2012 2:06PM----------------
 * Set the parameters for the motor overcurrent
 * protection system
 * --------------------------------------------------*/
static void csmSetMotorOCPParams(void)
{
    if ( console_GetArgCount() == 2 )
    {
        EEPROM_Param.nOvrCurrSmplThresh = console_GetNextParamDecimal();
        EEPROM_Param.nOvrCurrRevTimeMS10 = console_GetNextParamDecimal();
        ctrl_FixupOpParamValues(FALSE);  // false for clamp mode
        ctrl_SetEepromWriteFlag(TRUE);
    }
    console_Print( "Shade o/c params: %u [seq], %u [10ms] (%u [ms])\r\n",
        EEPROM_Param.nOvrCurrSmplThresh,
        EEPROM_Param.nOvrCurrRevTimeMS10,
        (10 * EEPROM_Param.nOvrCurrRevTimeMS10)
   );
}


/*---------------------------------------------------
 * Adjusts the value of the nThresh to match the legacy 
 * of 60-30-15 values, adjusted for the terminal client 
 * to use the proper value on the 2Nm and 4Nm 
 * --------------------------------------------------*/

UINT16 set_CSM2_OCTH(UINT16 nTsh)
{
    UINT8 deviceID = product_BSP_LoadDeviceModelType();

    if ( (deviceID == CSM_QMTDC_256_2) || (deviceID == CSM_QMTDC_250_2))              //2Nm Version
    {
        if ( nTsh > 20 )
        {
            return 0;
        }
        else
        {
            return (nTsh * 3);
        }
    }

  if ( (deviceID == CSM_QMTDC_256_4) || (deviceID == CSM_QMTDC_250_4) || (deviceID == CSM_QMTDC_275_4) )              //4Nm Version
    {
        if ( nTsh < 10 )
        {
            return 0;
        }
        else
        {
            return ((nTsh/2) * 3);
        }
    }

  return 0;
    
}


/*---------------------------------------------------
 * Return the proper value according to the type 
 * of motor (Smfy 6Nm, CSM2 2Nm, CSM2 4Nm)  
 * --------------------------------------------------*/

UINT16 displayOCTH()
{
    #ifndef CSM2
  
    return EEPROM_Param.nOvrCurrThresh10Nm;

    #else

    UINT8 deviceID = product_BSP_LoadDeviceModelType();

    if ( (deviceID == CSM_QMTDC_256_2) || (deviceID == CSM_QMTDC_250_2) )   //2Nm Version
    {
      return (EEPROM_Param.nOvrCurrThresh10Nm / 3);
    }
    if ( (deviceID == CSM_QMTDC_256_4) || (deviceID == CSM_QMTDC_250_4) || (deviceID == CSM_QMTDC_275_4 ))   //4Nm Version
    {
      return ((EEPROM_Param.nOvrCurrThresh10Nm / 3) * 2);
    }

    return 0;

    #endif
}


/*-----------------11/2/2012 3:42PM-----------------
 * Set the overcurrent threshold for the device
 * --------------------------------------------------*/
static void csmSetMotorOcThresh(void)
{
    if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
        UINT16 nThresh = console_GetNextParamDecimal(); // get the threshold parameter
        switch ( nThresh )
        {

            // the valid thresholds we can set
            #ifndef CSM2

            case 60:                    // 6.0 N-m
            case 30:                    // 3.0 N-m
            case 15:                    // 1.5 N-m
            
            #else

              #ifndef CSM_DRAPE_MOTOR
                case 40:
                case 20:
                case 10:
                case  5:
              #endif

              case  0:                    // Case that cancels any software overrcurrent threshold and only the hw threshold will be enabled

                  nThresh = set_CSM2_OCTH(nThresh);
                
            #endif

                EEPROM_Param.nOvrCurrThresh10Nm = nThresh;  // assign the value
                ctrl_FixupOpParamValues(FALSE);             // false for clamp mode
                ctrl_SetEepromWriteFlag(TRUE);              // set the write flag to save
                ctrl_SetSendOcThresh();                     // send the new level to the motor
                break;

            default:
                console_Print("Invalid threshold value (%u)\r\n", nThresh);
                break;
        }
    }
    console_Print("Motor overcurrent threshold %u\r\n", displayOCTH());
}

/*---------------------------------------------------
 * Generic function to move the shade to position 0 - 1000
 * --------------------------------------------------*/
static void csmGoto(UINT16 value)
{
    if ( value > 1000 )
    {        
        return;
    }


    if ( value == 1000 )
    {
        event_SetCnetJoins(CONTROL_ANAIN_SET_POSITION, 65535);        
    }     
    else
    {                                    
       //scale value to 16b
       value = (value * 65) + (value * 536 / 1000); 
       event_SetCnetJoins(CONTROL_ANAIN_SET_POSITION,value);
    }
     
    ErrLogAddEntryBase("@POS Set Cmd", 0, ERRLVL_NOTICE, value);   
}

static void csmStop(void)
{
    ShdPctrlStop();
    console_Print( "Stopping\r\n");
}


/*---------------------------------------------------
 * Console cmd to move the shade to position 1 - 999
 * --------------------------------------------------*/

static void csm2_GotoPosition(void)
{
    BOOL validValue = FALSE; 

    if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
        UINT16 value = console_GetNextParamDecimal();   // get the threshold parameter

        //Limit range to [0 - 1000]
        if ( value <= 1000 )
        {
           validValue = TRUE;
           csmGoto(value);                 
        }
    }
    
    if ( validValue == FALSE )
    {
        console_Print( "Valid values: 0 - 1000\r\n");
    }

}


/*---------------------------------------------------
 * Console cmd to set the shade speed
 * --------------------------------------------------*/
static void csm2_SetSpeed(void)
{
    if ( console_GetArgCount() == 1 )                 // allow setting if proper argument count
    {
        UINT16 value = console_GetNextParamDecimal(); // get the threshold parameter

        #if defined(CSM_DRAPE_MOTOR)
        if ( value < SHDCTRL_RPM_TENTH_MAX )
        #else
        if ( value < 310 )
        #endif
            event_SetCnetJoins(CONTROL_ANAIN_SET_SPEED, value);
    }
    else
        {
          #if defined(CSM_DRAPE_MOTOR)
            console_Print("Valid Ranges [6 - 50] or [100 - 500]\r\n");
          #else
            console_Print("Valid Ranges [10 - 30] or [100 - 309]\r\n");
          #endif
        }
}


/*---------------------------------------------------
 * Console cmd enter or exit LED test mode
 * --------------------------------------------------*/
static void csmLEDTestMode(void)
{
    if ( console_GetArgCount() == 1 )                 // allow setting if proper argument count
    {
        UINT16 value = console_GetNextParamDecimal(); // get the threshold parameter

        if ( value > 0 )
        {
            set_diag(TRUE);
            console_Print("LED Test Mode\r\n");
        }
        else
        {
            set_diag(FALSE);
            console_Print("Normal Mode\r\n");
        }
    }
    else
    {
        console_Print("LED Testmode is: %u\r\n",diag_IsActive());          
    }
    
}

/*---------------------------------------------------
 * Debug: Sends cmd to slave to exit Bootloader
 * --------------------------------------------------*/
static void csm_testSlaveExitBootloader(void)
{
   //ctrl_SetRequestMotorCmd(REQ_CMD_STARTUP);
   sfy_PutPacket( SFY_CMD_EXIT_BOOT );
   console_Print("Exit SL bootloader\r\n");
}



/**
* \brief   Test external flash chip operation
* \return  void
*/
#ifdef __MNET_DEVICE__
 #if !defined(CSM_QMTDC_163)&&!defined(CSM_DRAPE_MOTOR)

static void ConsoleCmdTestFlash(void)
{
//#if ( BLDOPT_APP_BTLDR_MODE == 1 )  // only external flash systems

    BOOL bResult, bExtendedInfo = FALSE;

    // console greeting; run the test with console output
    console_Print("Starting Test..." EOL);
    if ( console_GetArgCount() > 0 )
    {
        bExtendedInfo = console_IsParamStrMatch("DEBUG");
    }
    bResult = SelfTestExtFlashValidate( (bExtendedInfo) ? 0xFF : 0 );

    // print the overall test summary
    console_Print("Test %s" EOL, (bResult == TRUE)? "Passed" : "Failed" );
 
//#else
//    console_Print("Not supported on this platform (%u)" EOL, BLDOPT_APP_BTLDR_MODE );
//#endif
}

  #endif
#endif



#ifdef CSM2

/*---------------------------------------------------
 * No limits Motion test Mode with limits recovery 
 * on mode exit
 * --------------------------------------------------*/
static void csmMotionTest(void)
{
    static UINT32 tempUlimit = 0;
    static UINT32 tempLlimit = 0;
    static UINT8  tempsstate = 0;

    SHD_CMD_UPDATE sMotorData;
   // fetch the current motor operating data
    ShdDbgGetOpData(&sMotorData);

    if ( console_GetArgCount() == 1 )                 // allow setting if proper argument count
    {
        UINT16 value = console_GetNextParamDecimal(); // get the threshold parameter

        if ( value > 0 )
        {
           //save the current limits and state
            if(ctrl_GetShdOpState() == SS_NORMAL)
            {
                tempsstate = 1;
            }

            tempUlimit = EEPROM_Sys.sys_nUpperLimit;
            tempLlimit = EEPROM_Sys.sys_nLowerLimit;

           // Set the Upper and lower limit as CurrentPosition+- 2^21
           // Use proper adjustment for UINT24 math
            
            EEPROM_Sys.sys_nUpperLimit = math24bitAdd(sMotorData.position,2097152);
            EEPROM_Sys.sys_nLowerLimit = math24bitSub(sMotorData.position,2097152);
            ctrl_SetShdOpState( SS_NORMAL );
            //ctrl_SetEepromWriteFlag(TRUE); //No need to write the limits to EEPROM
           
            console_Print("Extended Limits Mode\r\n");
        }
        else
        {
            //On exit this mode, reestablished the limits and mode.
            EEPROM_Sys.sys_nUpperLimit = tempUlimit;
            EEPROM_Sys.sys_nLowerLimit = tempLlimit;
            if ( tempsstate == 1 )
            {
                ctrl_SetShdOpState( SS_NORMAL );
                console_Print("NORMAL Mode\r\n");
            }
            else
            {
                ctrl_SetShdOpState( SS_NO_LIMITS_SET );
                console_Print("NO_LIMITS Mode\r\n");
            }
           
        }
    }
        
}




/*---------------------------------------------------
 * CSM2 -> Set the upperlimit by amount of ticks from the Lower Limit
 * --------------------------------------------------*/
static void csm2SetULTicks(void)
{
    if ( console_GetArgCount() == 1 )                 // allow setting if proper argument count
    {
        UINT16 nticks = console_GetNextParamDecimal(); // get the threshold parameter

        if (( nticks > 0 )&&( ctrl_IsLimitValidLower() == TRUE ))
        {
            misc_ULTicks(nticks);

            console_Print("New UL Set: %d\r\n", EEPROM_Sys.sys_nUpperLimit);
        }
    }

}


/*---------------------------------------------------
 * CSM2 -> Set the RCB minimalist mode on/off
 * --------------------------------------------------*/
static void csm2_MinRCB(void)
{
    if ( console_GetArgCount() == 1 )                 // allow setting if proper argument count
    {
        UINT8 value = console_GetNextParamDecimal(); 

        if (value > 0)
        {
           set_usingSimplifiedRCB( TRUE );            
        }
        else
        {
           set_usingSimplifiedRCB( FALSE);   
        }
    }

    console_Print("MIN RCB is :%d\r\n", usingSimplifiedRCB());
}


/*
#warning MUST REMOVE, THIS IS ONLY FOR A DEBUG TEST

static void CSM2_MYTEST(void)
{
   misc_LL_RemoveTicks(1000);
   console_Print("LL Modified r\n");
} 
*/ 
 


/*---------------------------------------------------
 * CSM2 -> BLDC Eeprom test Trigger Fucntion
 * --------------------------------------------------*/
static void csmEEPROM_Test(void)
{
      console_Print("EEPROM Test\r\n");

      //Start EEPROM Test on BLDC
      #ifdef CSM2
         ctrl_SendQuery(67);
      #endif

      //Signal Test start on CSM
      csmEEPROM_TestStatus = 1;
#ifdef __MNET_DEVICE__
      //Store that test is initiated from the wireless console so the response can go over the air
      g_bGotCommandInRfrcon = LpConIsRconModeActive();
#endif
}

/*---------------------------------------------------
 * CSM2 cmd to get current avrg constntly
 * --------------------------------------------------*/
static void csm2GetAvrgCurrData(void)
{ 
    if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
       UINT8 state = console_GetNextParamDecimal();
       if ( state == 1  )
       {
           ctrl_SendQuery(140);
           csm2GetAvrgCurrentData = 1;
       }
       else
       {
           ctrl_SendQuery(141);
           csm2GetAvrgCurrentData = 0;
       }
    }


}
//@@@@
/*---------------------------------------------------
 * CSM2 cmd start an Advance ANgle calibration procedure 
 * Applies to CSM-QMTDC-163-1-CN/EX Only 
 * --------------------------------------------------*/
static void csm2AdvAngleCal(void)
{       
    startAdvAngleCalibration();
}


/*---------------------------------------------------
 * CSM2 cmd start an Advance ANgle calibration procedure 
 * Applies to CSM-QMTDC-163-1-CN/EX Only 
 * --------------------------------------------------*/
static void csm2OLAdvAngleCal(void)
{       
    start_OL_AdvAngleCalibration();
}


/*---------------------------------------------------
 * CSM2 cmd start an Field Advance Angle calibration procedure 
 * Applies to CSM-QMTDC-163-1-CN/EX Only 
 * --------------------------------------------------*/
static void csm2_Calibrate(void)
{       
    start_OL_FieldAdvAngleCalibration();
}
//@@@@
static void csm2_ForceSlaveFW(void)
{   
   EEPROM_Param.prm_forceSlaveFw = 21;
   appeeprom_WriteParamImageData();
   console_Print("PWR cycle device!\r\n");
}

/*---------------------------------------------------
 * Command to override the "Not communicating" state 
 * that should be normal during the slave upgrade. 
 * If thing go wrong with slave update, advance user/debuger/developper 
 * should use this command to establish a normal comunication flow
 * --------------------------------------------------*/
static void csm2DevStr(void)
{
    sendReportCresnetOk();
    CsmIsSlaveFW_Ready();   
}


/*---------------------------------------------------
 * CSM2 - Debug snippet for Testing Wiggle operation
 * --------------------------------------------------*/
/*
static void csm2_testwiggle(void)
{
    UINT8 val = 0;

    if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
        val = console_GetNextParamDecimal();
        if ( val == 0 )
        {
            val = FALSE;
        }
        else
            val = TRUE;

        wiggleSet(val);
    }

    console_Print("Wiggle is :%d\r\n",val);
} 
*/ 
 

static void csm2_stopCalibration(void)
{
    endAdvAngleCalibration();    
}

static void csm2Reverse(void)
{
    misc_Reverse();
    console_Print("Direction reversed\r\n");
}

static void csm2SetULimit(void)
{
    misc_setUpper();
    console_Print("UL Set\r\n");
}
  
static void csm2SetLLimit(void)
{
     misc_setLower();
     console_Print("LL Set\r\n");
}

static void csm2Get24BusVoltage(void)
{
    console_Print("Bus Voltage: %d\r\n",misc_get24BusVoltage());
}

/*
static void csm2SelectAntena(void)
{
    if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
       UINT8 antSelect = console_GetNextParamDecimal();
       // Csm2_SelectAntenna(antSelect);                // Was only used for testing, function will be disabled for safety
    }   
} 
*/ 

/*---------------------------------------------------
 * CSM2 - Debug tool for testing I2C Eeeprom operation
 * --------------------------------------------------*/
static void csm2_i2cEeprom(void)
{
    if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
       UINT8 operation = console_GetNextParamDecimal();
       misc_Runi2cEepromOperation(operation);
    }

}


/*---------------------------------------------------
 * CSM2 - Resets Slave CPU 
 * Apllicable to CSM-QMTDC- 163-1-CN/EX  and DRP-3-CN/EX 
 * --------------------------------------------------*/
static void csm2_resetSlave(void)
{
    hal_ResetSlave();
}

/*---------------------------------------------------
 * CSM2 - Makes Shade Device run at a very hig speed - intended for production only
 * Apllicable to CMS-QMTDC-25X-X-CN/EX CSM-QMTDC- 163-1-CN/EX  and DRP-3-CN/EX 
 * --------------------------------------------------*/
static void csm2_prodSpeedMode(void)
{   
   ctrl_SendQuery(Q_TOGGLE_PRODUCTION_MODE);            
}

#ifdef __MNET_DEVICE__ 
static void nvInfoEx(void)
{    
    //This call to NvDataCommand is only active for 1NmEX and DrapeEX  
     
       #if defined(CSM_QMTDC_163)||defined(CSM_DRAPE_MOTOR)       
        csm2_NvDataCommand();
       #endif
       
}
#endif

#ifdef __MNET_DEVICE__ 
static void restoreEx(void)
{
     //This call to NvDataCommand is only active for 1NmEX and DrapeEX  
            
        console_Print( "Restoring... please wait" );      
        appeeprom_ApplicationRestore();       
      
}
#endif  

#ifdef __MNET_DEVICE__ 
static void getModelVersion(void)
{
    //check if the model is international
    if (isInternational() == TRUE )
    {
        console_Print( "International Version\r\n" );
    }
    else
    {
        console_Print( "North America Version\r\n" );
    }      
}
#endif  

/*---------------------------------------------------
 * CSM2 - Counts Unhandled Packets
 * --------------------------------------------------*/
static void csm2_getBadPktCount(void)
{
   console_Print("Bad Pkt Count: %d\r\n", getBadPacketCount());
}

/*---------------------------------------------------
 * CSM2 - Debug: Sends cmd to slave to jump into Bootloader
 * --------------------------------------------------*/
static void csm_testSlaveToBootloader(void)
{
   sfy_PutPacket( SFY_CMD_ENTER_BOOT );
   console_Print("Slave -> Bootloader\r\n"); 
}

static void csm2getFlashCycles(void)
{
   console_Print("Flash Writes: \r\n");
   csm2SetConsoleQuery(1); 
   ctrl_SendQuery(Q_GET_FLASH_WRITES);
}


static void csm2setSlowSpeedOverride(void)
{
   if ( console_GetArgCount() == 1 )                  
    {
       UINT8 value = console_GetNextParamDecimal();
       misc_setLowspeedEnable(value);
    }
   
   console_Print("Slow Speed Enable: %d\r\n", misc_getLowspeedEnable());
}

static void csm2_OLstartupPower(void)
{
   UINT8 value = 0;

   if ( console_GetArgCount() == 1 )                  
    {
       value = console_GetNextParamDecimal();

       if (value == 1) 
       {
          ctrl_SendQuery(177);  // Set to LOW startup
       }
       else
       {
          ctrl_SendQuery(176);  // Set to High
       }
      
    }
   else
   {
      ctrl_SendQuery(109);
      value = getBLDC_OL_Startup();
   }
   
   if (value < 2)  // Cuse we dont want to show the default value or a wrong one. 
   {
      console_Print("OL Startup: %d\r\n", value);
   }
   

}

static void csm2_StopThresh(void)
{
   ShdPctrl_PrintOL_stopTresh();
}

static void csm2_SetStopThreshDebug()
{
   UINT8 value;

   if ( console_GetArgCount() == 1 )                  
    {
       value = console_GetNextParamDecimal();

       ShdPctrl_SetStopThreshDebug(value);             
    }
  
}


/*-----------------10/7/2013 12:19PM - LR ----------------
 *  CSM2  - Wil display info about adv angle calibration 
 *  DRAPE - Function to display BLDC MOTOR MPOCTH configuration
 *
 * --------------------------------------------------*/
static void drape_octh(void)
{
   ShdDbgBLDC_DataUpdate(21);   //Signal main loop to start a Data acquire mode
}


#ifdef CSM_DRAPE_MOTOR

/*---------------------------------------------------
 * DRAPE - Finds Close Hard Limit
 * --------------------------------------------------*/
static void drape_FindCloseLimit(void)
{
    DrapeFindCloseLimit(0);
}

/*---------------------------------------------------
 * DRAPE - Finds Open Hard Limit
 * --------------------------------------------------*/
static void drape_FindOpenLimit(void)
{
    DrapeFindOpenLimit();
}

/*---------------------------------------------------
 * DRAPE - Finds Both Hard limits automatically
 * --------------------------------------------------*/
static void drape_FindLimits(void)
{    
    DrapeSetState(DRAPE_STATE_FINDLIMITS);
}

/*---------------------------------------------------
 * DRAPE - Set current position as the Position Reference 
 * Use: For testing a reference position. 
 * --------------------------------------------------*/
static void drape_SetRef(void)
{
    DrapeSetPositionKnown();
    ctrl_SendQuery(Q_RESET_ENCODER_MID_RUN);      //Indicates to BLDC that it should reset Encoder to middle run.
}



/*---------------------------------------------------
 * DRAPE - Debug Command/Function for viewing Drape param log 
 * Will only work if enabled for R&D 
 * --------------------------------------------------*/
static void drape_ShowLog(void)
{
    DrapeShowLogData();
}

/*---------------------------------------------------
 * DRAPE - Debug Command/Function for clearing Drape param log 
 * Will only work if enabled for R&D 
 * --------------------------------------------------*/
static void drape_ClearLog(void)
{
    DrapeClearLogData2();
}


/*---------------------------------------------------
 * DRAPE - Triggers a Touch Sensitivity Capture
 * --------------------------------------------------*/
static void drape_CaptureTouch(void)
{
   Drape_TouchSetMode();
}

/*---------------------------------------------------
 * DRAPE - Enables or disables Touch Reaction
 * --------------------------------------------------*/
static void drape_setTouch(void)
{
    if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
       UINT8 value = console_GetNextParamDecimal();
       DrapeSetTouch(value);
    }

    console_Print("Touch is: %d\r\n",DrapeGetTouchState() );

}

/*---------------------------------------------------
 * DRAPE - Enables or Disables the backdoor latch 
 *         for going faster than 50 RPM
 * --------------------------------------------------*/
static void drape_HighSpeed(void)
{
    if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
       UINT8 value = console_GetNextParamDecimal();
       DrapeSetSpeedLatch(value);
    }

    console_Print("High Speed Mode is: %d\r\n",DrapeGetSpeedLatch() );
}


/*---------------------------------------------------
 * DRAPE - For Entering Debug Mode -  Helps to 
 *         troubleshoot the touch feature 
 * --------------------------------------------------*/
static void drape_Debug(void)
{
    if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
       UINT8 value = console_GetNextParamDecimal();
       Drape_DebugFlag(value);
    }

    console_Print("Drape Debug: %d\r\n",DrapeGetDebug() );
}


/*---------------------------------------------------
 * DRAPE - Sets the Touch sensitivity from console
 * --------------------------------------------------*/
static void drape_setTouchSensitivity(void)
{
    if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
       UINT8 value = console_GetNextParamDecimal();
       DrapeSetTouchSensitivity(DrapeUserSensitivity2Param(value));
    }
    
    console_Print("Touch Sensitivity: %d\r\n",DrapeGetUserSensitivity() );

    if ( DrapeGetDebug() != 0 )
    {
        console_Print("Raw Value: %d\r\n",DrapeGetSensitivity() );
    }

}

/*---------------------------------------------------
 * DRAPE - Sets the Touch Mode (This is for R&D and Debug)
 * --------------------------------------------------*/
static void drape_TouchMode(void)
{
    if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
       UINT8 mode = console_GetNextParamDecimal();
       DrapeSetTouchMode(mode);
    }
}

/*---------------------------------------------------
 * DRAPE - Triggers a Speed capture procedure (Set the speed by moving the fabric)
 * --------------------------------------------------*/
static void drape_CaptureSpeed(void)
{
    Drape_SpeedSetMode();
}



#endif
 

/* Debug functions for testing MC Group id settings
static void csm2TestSetGroupID(void)
{
     sSfyJoins.sSlot20.nGroupID[0] = 5;
     sSfyJoins.sSlot20.nGroupID[1] = 7;
     sSfyJoins.sSlot20.nGroupID[2] = 9;
     sSfyJoins.sSlot20.nGroupID[3] = 11;
     sSfyJoins.sSlot20.nGroupID[4] = 13;
     sSfyJoins.sSlot20.nGroupID[5] = 15;         
}

static void csm2PrintGroupID(void)
{
    for (int i=0;i<6;i++)
    {
         console_Print("Grp ID %d : %d\r\n",i, sSfyJoins.sSlot20.nGroupID[i]);
    }
} 
*/ 


/*---------------------------------------------------
 * CSM2 - Triggers getting a given Slave debug buffer
 * --------------------------------------------------*/
void csm2ReqBLDC_Buffer(UINT8 buffer, char* name)
{
     console_Print("Getting %s\r\n",name);
     csm2GetCurrentData = buffer;
     csm2CountCurrentData = 0;
}

/*---------------------------------------------------
 * CSM2 Current Buffer data display Trigger Function
 * --------------------------------------------------*/
static void csm2GetBuffer(void)
{
    // Obuffer is 2         //PID control values
    // Mbuffer is 3         //PID control values
    // Nbuffer is 4         //PID control values

    // Hall Buffer is 5
    // Hall Duration is 6
    // Sbuffer is 7            // Stop Log - Why the motor stoped or events happening
    // Rbuffer is 8
    // TBuffer is 9
    // FBuffer is 10           // Trace of input speeds 

    if ( console_GetArgCount() == 1 )                 // allow setting if proper argument count
    {
        UINT16 value = console_GetNextParamDecimal(); // get the threshold parameter

        if ( value > 1 )
        {
            csm2ReqBLDC_Buffer(value, "Buffer " + value);       
        }
    }
}


/*---------------------------------------------------
 * CSM2 Current Buffer data display Trigger Function
 * --------------------------------------------------*/
static void csm2GetCurrData(void)
{
     csm2ReqBLDC_Buffer(1, "I");
}

/*---------------------------------------------------
 * CSM2 hfm data
 * --------------------------------------------------*/
/*
static void csm2GetHFMDataArray(void)
{
     ctrl_SendQuery(99);              //Resets the HFM index to 0 to start reading the first data point
     console_Print("HFM data\r\n");
     csm2GetHFM = 1;
     csm2GetHFMData = 0;
     ticktimer_Clear(TIMERS_BLDC_CURRENTDATA);
} 
*/ 


/*---------------------------------------------------
 * CSM2 Autocalibration Trigger Function
 * --------------------------------------------------*/
static void csm2AutoCalibrate(void)
{
   
    //Make sure limits are set first!
    if ((ctrl_IsLimitValidUpper()== TRUE) && (ctrl_IsLimitValidLower()==TRUE))
    {
         
     //TODO: UI Buttons should be Disabled while csm2AutoCal > 0 

     console_Print("OCTH Autocalibration\r\n");
          
     //Set OCTH to 0 ->BLDC
     setCSM2AutoCal(1);
     EEPROM_Param.nOvrCurrThresh10Nm = 0;        // assign the value
     ctrl_FixupOpParamValues(FALSE);             // false for clamp mode
     ctrl_SetEepromWriteFlag(TRUE);              // set the write flag to save                             
     ctrl_SetSendOcThresh();                     // send the new level to the motor
     ticktimer_Clear(TIMERS_AUTOCAL);
    }

    else
    {
     console_Print("Set limits\r\n");
    }
}

/*---------------------------------------------------
 * CSM2 Enables / Disables the OCTH Autocalibration feature
 * --------------------------------------------------*/
static void csm2EnableDAutocal(void)
{
    if ( console_GetArgCount() == 1 )                 // allow setting if proper argument count
    {
        UINT16 value = console_GetNextParamDecimal(); // get the threshold parameter

        CSM2_setDAutocalOn((UINT8)value);     
    }
    
    console_Print("Dynamic OCTH is ");
    if ( CSM2_isDAutocalOn() == FALSE )
    {
       console_Print("OFF\r\n");
    }
    else
       console_Print("ON\r\n");

}


static void csm2_suppressTimeBWRcb(void)
{
   if ( console_GetArgCount() == 1 )                 
    {
        UINT16 value = console_GetNextParamDecimal(); 

        shade_rcb_fb_suppressTimeBetween( (UINT8)value);
   }

}

// Only valid for 1Nm shade
static void csm2_downMode(void)
{
    if ( console_GetArgCount() == 1 )                 
    {
        UINT16 value = console_GetNextParamDecimal(); 

        if ( value == 0 )
        {
           ctrl_SendQuery(143);    // Sets down mode to Sine
        }
        else
        {
           ctrl_SendQuery(144);   // Sets down mode to Trapezoidal 
        }
    }
}


static void csmIsSlaveFWValid(void)
{
    #ifdef CSM2
     #if !defined(CSM_QMTDC_163)&&!defined(CSM_DRAPE_MOTOR)
      #ifdef __MNET_DEVICE__     

    /* DO NOT REMOVE, THIS IS FOR TESTING AND DEBUG
    BLDC_app_GetFromEEPROM(BLDC_APP_SIZE - 256);  // 36608);

    UINT32 num = htonl(*((UINT32 *)&BLDC_app[252]));     // Alternative cast - Use the htons for out known Endianes
    UINT32 num2 = htonl(*((UINT32 *)&BLDC_app[248]));         
    
    console_Print("Code CK_VALUE: %u  EEPROM CK_VALUE: %u  \r\n", BLDC_CK_Value,  num);
    console_Print("Code CK_VALUE2: %u  EEPROM CK_VALUE2: %u  \r\n", BLDC_CK_Value2,  num2 );
 
    console_Print("Motor Ver: %d  BLDC Ver: %d \r\n", ctrl_GetMotorFirmwareVersion(), BLDC_version );
    console_Print("Motor Rev: %d  BLDC Rev: %d \r\n", ctrl_U32_GetMotorFirmwareRev(), BLDC_revision );
    */ 
     
    console_Print("Slave FW "); 

    if (  CbIsExtendedFwValid() == TRUE )
       {
         console_Print("valid\r\n");
       }
       else
       {
         console_Print("not valid\r\n");
       }
    
     #endif

    #else
       csm2SetConsoleQuery(1);
       ctrl_SendQuery(Q_FIRMWARE_VERSION);      // Requests to get back the slave fw revision

    #endif
   #endif
}

static void csm2_GetCalibrationData(void)
{
    #ifdef CSM2
     #ifdef CSM_QMTDC_163
    
    console_PutString( "Current Calibration Data\r\n" );
    ShdDbgBLDC_DataUpdate(23);   //Signal main loop to start a BLDC Data acquire mode
    /*ctrl_SendQuery(Q_GET_AA_CW);
    ctrl_SendQuery(Q_GET_AA_CCW);
    ctrl_SendQuery(Q_GET_AA_CW_B);
    ctrl_SendQuery(Q_GET_AA_CCW_B);
    ctrl_SendQuery(Q_GET_AA_CW_C);
    ctrl_SendQuery(Q_GET_AA_CCW_C);
    ctrl_SendQuery(Q_GET_CW_OPTIMUM_ANGLE);
    ctrl_SendQuery(Q_GET_CCW_OPTIMUM_ANGLE);
    ctrl_SendQuery(Q_GET_TARGET_SPEED);*/
    
    
#endif
#endif
}

/*-----------------1/1/201 9:58AM - LR ----------------
 *  Will return the state of the digital output join - Mainly for debugging
 *
 * --------------------------------------------------*/
static void csmGetJoinState(void)
{
    BOOL valid = FALSE;
    BOOL joinState = FALSE;

    if ( console_GetArgCount() == 2 )                    // allow setting if proper argument count
    {
        //First param is the Slot
        //Second param is the Digital Join Out

        UINT8 slot, djoin;

        slot  = console_GetNextParamDecimal();
        djoin = console_GetNextParamDecimal();

        if ( slot == 1 )
        {
            valid = TRUE;

            switch ( djoin )
            {
               case 1: joinState = sSfyJoins.sSlot1.bIsRaising;
                       break;
               case 2: joinState = sSfyJoins.sSlot1.bIsLowering;
                       break;
               case 3: joinState = sSfyJoins.sSlot1.bIsFullOpen;
                       break;
               case 4: joinState = sSfyJoins.sSlot1.bIsFullClosed;
                       break;
               case 5: joinState = sSfyJoins.sSlot1.bIsStopped;
                       break;
               case 6: joinState = sSfyJoins.sSlot1.bLastDirOpen;
                       break;
               case 7: joinState = sSfyJoins.sSlot1.bLastDirClosed;
                       break;
               default:valid = FALSE;
                       break; 

            }
        }

        if ( slot == 2 )
        {
            valid = TRUE;

            switch ( djoin )
            {
               case 5: joinState = sSfyJoins.sSlot2.bUpperLimSet;
                       break;
               case 6: joinState = sSfyJoins.sSlot2.bLowerLimSet;
                       break;
               case 7: joinState = sSfyJoins.sSlot2.bRaiseIsClkwise;
                       break;

               #ifdef CSM_DRAPE_MOTOR

               case 10: joinState = sSfyJoins.sSlot2.bfindingLimits;
                        break;

               #endif

               default:valid = FALSE;
                       break;
            }
        }

        if ( valid == TRUE )
        {
            console_Print("Join State: (%u)\r\n", joinState);
        }

    }
}


#if 0
/*-----------------1/28/2014 9:58AM - LR ----------------
 *  Will return values stored in EEPROM for the given address
 *  Only for debugging.
 * --------------------------------------------------*/
static void csmDebugReadEEPROM()
{
    if ( console_GetArgCount() == 1 )                    // allow setting if proper argument count
    {
        //First param is the Slot
        //Second param is the Digital Join Out

        UINT8 addr;
        INT16 i;

        addr  = console_GetNextParamDecimal();

        BLDC_app_GetFromEEPROM((UINT16)(addr * 256));
                      
        for (i=0; i < 256; i++)
        {
           console_Print("%x ", BLDC_app[i]);
        }

    }
}

#endif


/*-----------------5/10/2013 2:19PM - LR ----------------
 *  Function to override the BLDC Driver's gear ratio configuration
 *
 * --------------------------------------------------*/
static void csm2OvrConfig(void)
{   

    BOOL valid = FALSE;

    if ( console_GetArgCount() == 1 )                    // allow setting if proper argument count
    {
        UINT8 selection = console_GetNextParamDecimal(); // get the new configuration

        if ( selection < 8 )
        {
            valid = TRUE;

            #if !defined(CSM_DRAPE_MOTOR)&&!defined(CSM_QMTDC_163)
            
            ctrl_SetSendOvrConfig(selection);

            switch (selection)
            {
               case 3: //250 2Nm 
                   appeeprom_WriteDeviceModelType(DEVICE_2NM_250TUBE);
                   break;
               case 4: //256 4Nm
                   appeeprom_WriteDeviceModelType(DEVICE_4NM_256TUBE);
                   break;
               case 5: //250 4Nm
                   appeeprom_WriteDeviceModelType(DEVICE_4NM_250TUBE);
                   break;

               case 6: //275 4Nm
                   appeeprom_WriteDeviceModelType(DEVICE_4NM_275TUBE);
                   break; 

               case 7: //250 4Nm SL
                   appeeprom_WriteDeviceModelType(DEVICE_4NM_250TUBE_SL);
                   break; 

               case 2: //256 2Nm
               default:
                   appeeprom_WriteDeviceModelType(DEVICE_2NM_256TUBE);
                   break;
            }

            #else
     
             #ifdef CSM_QMTDC_163

             switch (selection)
             {

               case 2: //1Nm Hsheer 
                   appeeprom_WriteDeviceModelType(DEVICE_1NM_HSHEER);
                   silhouette_Enable( TRUE );
                   break;
               
               case 1: //1Nm Regular 
               default:
                   appeeprom_WriteDeviceModelType(DEVICE_1NM_REGULAR);
                   silhouette_Enable( FALSE );
                   break;
             }

             #endif

            #endif
        }

        console_Print("Reboot Device!\r\n");
    }
    

    if ( valid == FALSE )
        console_Print("Valid value[0 - 7]\r\n");

    

}

#ifdef USE_MIDPOINT_CORRECTION

/*-----------------8/5/2013 3:16PM - LR ----------------
 * Will set the Traking mode for the current shade. 
 * 0 - Master: Will behave normal, no position corrections 
 * 1 - Slave : Shade will have its position corrected according to a master data reference 
 * --------------------------------------------------*/
static void csm2_TrackingMode(void)
{
    
 if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
        UINT16 value = console_GetNextParamDecimal(); // get the threshold parameter
        switch ( value )
        {
           case 0: EEPROM_Param.prm_TrackingMode = TRACK_MASTER;   
                   break;

           case 1: EEPROM_Param.prm_TrackingMode = TRACK_SLAVE; 
                   break;

           default:
                   EEPROM_Param.prm_TrackingMode = TRACK_MASTER;
                   break;
        }
                
    }

    console_Print("Tracking Mode: ");
    if ( EEPROM_Param.prm_TrackingMode == TRACK_SLAVE )
    {
      console_Print("Slave\r\n" );
    }
    else
    {
      console_Print("Master\r\n" );
    } 
     

}


/*-----------------8/5/2013 3:16PM - LR ----------------
 * Calculates the necesary compensation parameters to make this slave shade track a master profile 
 *  
 * --------------------------------------------------*/
static void csm2_SetMidPointCorrection(void)
{
    
    if ( EEPROM_Param.prm_TrackingMode == TRACK_SLAVE )
    {
        console_Print("Pos is: %u\r\n", ctrl_GetNowPosition());
        
        //Scale to proper limits
        SHD_CMD_UPDATE sMotorData;
        UINT32 pthou;
        UINT16 midPointMatch;

        // fetch the current motor operating data
        ShdDbgGetOpData(&sMotorData);

        // find the shade position as an analog level
        midPointMatch = ShdLimGetLevelFromPosition(EEPROM_Sys.sys_nUpperLimit, EEPROM_Sys.sys_nLowerLimit, sMotorData.position);
        pthou =  ((1000 * (UINT32)midPointMatch) / 65535);

        console_Print("L16 Pos is: %u\r\n", midPointMatch );
        console_Print("%Pos is: %u\r\n", pthou/10 );       
        console_Print("Setting as Midpoint\r\n");
       
        csm_shadeTracking_calculateAB(midPointMatch);

        console_Print("A = %d and B = %d\r\n",csm_shadeTracking_getA(),csm_shadeTracking_getB());
       
    }
    else
    {
        console_Print("in Master Mode\r\n");
    } 
     
     
}


/*-----------------3/18/2014 3:16PM - LR ----------------
 * Sets internal parameters for Mid Point Correctin algorithm ... the relation of turns from Master to Slave
 *  
 * --------------------------------------------------*/

static void csm2_SetMPC_Params()
{
    if ( console_GetArgCount() == 2 )                   // allow setting if proper argument count
    {
        UINT16 value1 = console_GetNextParamDecimal(); // 
        UINT16 value2 = console_GetNextParamDecimal(); //

        csm_shadeTracking_setMasterTurnsRNum(value1);
        csm_shadeTracking_setMasterTurnsRDiv(value2);

        console_Print("MPC params changed\r\n");

    }
    else
    {
        //Just show the current MPC Params
         console_Print("MPC params: M2S Num: %d, M2S Div: %d\r\n",csm_shadeTracking_getMasterTurnsRNum(),csm_shadeTracking_getMasterTurnsRDiv());
    }
}

#endif



/*-----------------10/7/2013 12:19PM - LR ----------------
 *  Function to display BLDC MOTOR configuration
 *
 * --------------------------------------------------*/
static void csm2GetBldcData(void)
{
   ShdDbgBLDC_DataUpdate(1);   //Signal main loop to start a BLDC Data acquire mode
}


/*-----------------4/7/2014 10:22PM - LR ----------------
 *  Function to display BLDC MOTOR Error Debuging
 *
 * --------------------------------------------------*/
static void csm2GetBldcDebug(void)
{
   ShdDbgBLDC_DataUpdate(10);   //Signal main loop to start a BLDC Data acquire for BLDC Debug Logic
}


static void csm2_DC_OVR(void)
{
    if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
       UINT8 state = console_GetNextParamDecimal();
       if ( state == 1  )
       {
           ctrl_SendQuery(59);
       }
       else
           ctrl_SendQuery(60);
    }
}

/*-----------------3/8/2013 2:29PM - LR ----------------
 * General Query command, used for getting some ADC
 * values from BLDC driver
 * --------------------------------------------------*/
static void csm2GeneralQuery(void)
{
    if ( console_GetArgCount() == 1 )                   // allow setting if proper argument count
    {
        UINT8 selection = console_GetNextParamDecimal(); // get the threshold parameter
        csm2SetConsoleQuery(1);
        ctrl_SendQuery(selection);
    }
#ifdef __MNET_DEVICE__    
    //Store that test is initiated from the wireless console so the response can go over the air
      g_bGotCommandInRfrcon = LpConIsRconModeActive();
#endif
}


/*-----------------3/8/2017 2:29PM - LR ----------------
 * Siluette/ Tilt UI functions
 * 
 * --------------------------------------------------*/
static void csm2MoveTicks(void)
{
    #ifdef CSM_QMTDC_163
    if ( console_GetArgCount() == 2 )                   
    {
        UINT8 dir    = console_GetNextParamDecimal();      
        UINT16 ticks = console_GetNextParamDecimal();      
        ctrl_SendSendSilhTicks( dir, ticks );
    }
    #endif
}

static void csm2TiltOpen(void)
{
   silhouette_Open();
}

static void csm2TiltClose(void)
{
   silhouette_Close();
}

static void csm2TiltEnable(void)
{
   if ( console_GetArgCount() == 1 )                   
    {
       UINT8 value    = console_GetNextParamDecimal();      
       
       silhouette_Enable( value);          
    } 
   else
   {
      console_Print("Tilt Enabled: %s\r\n", isSilhouetteEnabled()?"True":"False");
   }
}

static void csm2TiltPos(void)
{
   if ( console_GetArgCount() == 1 )                   
   {
        UINT8 pos = console_GetNextParamDecimal();
        
        silhouette_SetPosition( pos );
   }
   else
   {
      console_Print("Tilt Pos: %d\r\n", silhouette_GetPosition());
   }
}

// Use for adjusting the Tilt offset units
static void csm2TiltLimit(void)
{
   if ( console_GetArgCount() == 1 )                   
   {
        #ifdef __MNET_DEVICE__
        
        TILT_ADJUST_TYPE pos = console_GetNextParamDecimal();

        silhouette_SetTiltLimit( pos );

        #endif


        #ifdef __CNET_DEVICE__

        TILT_ADJUST_TYPE pos = console_GetNextParamSignedLong();
                
        silhouette_SetTiltLimit( pos );
        
        #endif        
   }
   else
   {
      #ifdef __MNET_DEVICE__

        console_Print("Tilt Offset Units: %d\r\n", silhouette_GetOffsetUnits_Mnet());

      #else

        console_Print("Tilt Offset Units: %d\r\n", silhouette_GetOffsetUnits());

      #endif
   }

}


/*-----------------3/28/2013 9:40AM - LR ----------------
 * Adjusting the speed with a fraccional value
 * intended for CSM2 - experimental
 * --------------------------------------------------*/
/*
static void csm2SpeedAdjustUp(void)
{
    UINT8 fraccRPM = ctrl_GetNowFraccSpeed();
    INT8 intRPM   = ctrl_GetSpeedIntAdj();

    if ( fraccRPM == 9 )
    {
      ctrl_SetNowFraccSpeed(0);
      intRPM++;
      ctrl_SetSpeedIntAdj(intRPM);
    }
    else
    {
      fraccRPM++;
      ctrl_SetNowFraccSpeed(fraccRPM);
    }

    console_Print("Speed Adjust(%d , %u)\r\n", ctrl_GetSpeedIntAdj(), ctrl_GetNowFraccSpeed());
}
*/
/*-----------------3/28/2013 9:40AM - LR ----------------
 * Adjusting the speed with a fraccional value
 * intended for CSM2 - experimental
 * --------------------------------------------------*/
/*
static void csm2SpeedAdjustDn(void)
{
    UINT8 fraccRPM = ctrl_GetNowFraccSpeed();
    INT16 intRPM   = ctrl_GetSpeedIntAdj();

    if ( fraccRPM == 0 )
    {
      ctrl_SetNowFraccSpeed(9);
      intRPM--;
      ctrl_SetSpeedIntAdj(intRPM);
    }
    else
    {
      fraccRPM--;
      ctrl_SetNowFraccSpeed(fraccRPM);
    }

    console_Print("Speed Adjust(%d , %u)\r\n", ctrl_GetSpeedIntAdj(), ctrl_GetNowFraccSpeed());

}
*/

/*-----------------4/16/2013 9:40AM - LR ----------------
 * Help comand to read and verify value of Fraccional RPM
 * --------------------------------------------------*/
/*
static void csm2GetFraccRPM(void)
{
   console_Print("Fracc RPM (%u )\r\n", ctrl_GetNowFraccSpeed());
}
*/

#endif



/*! The table of command names */
static char const * g_pszcCmdNames[] =
{
    "MASSERASE",
    "SHOWUI",
    "MDBGBINS",
    "MDBGCNTR",
    "MDBGMASK",
    "MDBGINFO",
    "MDBGLOGS",
    "MDBGINIT",
    "MDBGCOMP",
    "MDBGOCPD",
    "SNVREAD",
    "SETJOGTM",
    "SETKICKTM",
    "SETOCPPRM",
    "SETOCTHRESH",
    "GOTOPOS",
    "SETSPEED",
    "TESTLEDS",
    "STOP",
    
    #ifdef __MNET_DEVICE__     
     #if !defined(CSM_QMTDC_163)&&!defined(CSM_DRAPE_MOTOR)   //Intended for CSM-2NmEx and CSM-SomfyEx that have External SPI flash
     "TESTFLASH",
     #endif
    #endif


#ifdef CSM2
    "SLBOOT",                // Makes Slve enter bootloader
    "ETBOOT",
    "TESTMOTION",
    "QUERY",
    "OVRCONFIG",
    "BLDCDATA",
    "TESTEEPROM",
    "CURRENTBUFFER",
    "GETBUFFER",
    "AUTOCAL",
    "JOINSTATE",             //This is mainly for Debugging ... So take out if not needed 
    "SLAVE",                 // 
    "ULTICKS",    
    "DAUTOCAL",
    "BLDCDEBUG",
    "DCOVR",
    "AVRGI",
    "ADVANGLECAL",
    "SETREVERSE",
    "SETUPPER",
    "SETLOWER",
    "GET24BUS",
    "I2CEEPROM",
    "SLRESET",
    "BADPKT",
    "SCAL",
    "PROD",
    "DOWNMODE",
    "MOVETICKS",
    "TILTOPEN",
    "TILTCLOSE",
    "TILTENABLE",
    "TILTPOS",
    "TILTMAXTICS",
    "OLANGLECAL", //production uses this command for fw older than 238
    "DEVSTR",
    "FLASH",
    "SETSLOW",
    "STARTUP",
    "STPTHRESH",
    "DBGSTPTHRESH",
#ifdef CSM_DRAPE_MOTOR
    "DRAPEOCTH",
#else
    "OLOFFSET",
#endif
    "CALIBRATE",
    "FORCEFW",
    "MINRCB",
	"GETCALIBDATA",
    //"MYTEST",
    
    #ifdef __CNET_DEVICE__
     "RCBTIME",
    #endif
   
     #ifdef __MNET_DEVICE__
    "RESTORE",
    "NVDATA",
    "MODELTYPE",
     #endif 
     
     #ifdef __CNET_DEVICE__
       "DBGRSU",
     #endif
     
  #ifdef USE_MIDPOINT_CORRECTION
    "TRACKMODE",
    "SETMIDPOINTCORR",
    "SETMPCPARAMS",
  #endif 

  #ifdef CSM_DRAPE_MOTOR
    "FINDCLOSE",
    "FINDOPEN",
    "DRPLOG",
    "DRPCLRLOG",
    "TOUCHSENSE",
    "TOUCHMODE",
    "TOUCHSPEED",
    "FINDLIMITS",
    "DRAPEREF",
    "SETTOUCH",
    "TOUCH",
    "HIGHSPEED",
    "DRPDEBUG",
    
  #endif

#endif

   #ifdef __CNET_DEVICE__
    #ifdef CSM2
    "RCBLOG",
    "RCBRPT",
    #endif
   #endif
    
};

/*! The table of command help strings */
static char const * g_pszcCmdHelp[] =
{
    "Mass erase eeprom",
    "Display UI state",
    "Display motor limits histogram data",
    "Display debug counters",
    "<hex8b> Set/display motor debug mask",
    "Display settings and operation data",
    "Display error event log data",
    "Reset motor error capture storage",
    "Display overshoot compensation data",
    "Display overcurrent protection data",
    "Read motor NV data",
    "<10ms> Set motor jog time",
    "<10ms> Set motor jog direction reversal time",
    "<seq><revTm10ms> Set overcurrent protection parameters",
    "<thresh x 10> Set overcurrent threshold in [N-m] x 10",
    "Go to position",
    "Set Speed [10-30] or [100-300]",
    "LED TestMode (on->1, off->0)", 
    "Stop",
      
    #ifdef __MNET_DEVICE__
     #if !defined(CSM_QMTDC_163)&&!defined(CSM_DRAPE_MOTOR)   //Intended for CSM-2NmEx and CSM-SomfyEx that have External SPI flash
     "Test Ext. Flash",
     #endif
    #endif
             
    
#ifdef CSM2
    "", 
    "", 
    "Extended limits test",
    "Req BLDC Variables",    
    "Overrides gear ratio config",
    "BLDC Motor config",
    "EEPROM Test",
    "BLDC I data",
    "Data Buffer"    
    "OCTH AutoCalibration",
    "Show Join States",
    "Test Slave Valid FW",
    "Sets UL by ticks amount",   
    "Dynamic OCTH on/off",
    "BLDC Debug",
    "DC Override",
    "I Average",
    "Adv Angle Cal.",
    "Reverse Motor",
    "Set UL",
    "Set LL",
    "Get Bus V", 
    "Test IC2 eeprom",
    "Reset Slave",  
    "Bad PKT count",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "", 
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
	"Get Calibration Data",
    //"",
    
    #ifdef __CNET_DEVICE__  
     "RCB SUPPRESS TIME",
    #endif  

     #ifdef __MNET_DEVICE__
    "Factory Restore",
    "NV Data/info", 
    "US/International",
     #endif

     #ifdef __CNET_DEVICE__
       "resync debug",
     #endif
   
    
  #ifdef USE_MIDPOINT_CORRECTION
    "Sets as Master or Slave",
    "Sets Midpoint correction",
    "Sets MPC Parameters",
  #endif  

  #ifdef CSM_DRAPE_MOTOR
    "FINDS CLOSE LIM",
    "FINDS OPEN LIM",
    "",
    "",
    "Touch Set Mode",
    "Touch Reaction Mode",
    "Speed Set Mode",
    "",   
    "",  //sETS DRAPE close limit REFERENCE    
    "Sets Touch sensitivity",
    "Touch on[1], off[0]",
    "Enables up to 75 RPM",
    "",
  #endif
        

#endif

  #ifdef __CNET_DEVICE__
    #ifdef CSM2
    "<reset> Display or clear RCB send log",
    "<on|off|set> <base, max> Display or set dynamic RCB reporting parameters",
    #endif
  #endif

};

/*! Make a table the command pointers */
static void (*g_pCmdFunc[])(void) =
{
    csmMassEraseEEPROM,
    csmShowUiState,
    ShdDbgConRptHistData,
    ShdDbgConDumpCounters,
    OnCmdMotorDebugMask,
    csmRtDbgConsoleOpData,
    ShdDbgConRptCaptureData,
    ShdDbgInitialize,
    ShdPctrlOvrshtCompDebug,
    csmDisplayMotorOCPD,
    sfy_Nvd_StartReadSequence,
    csmSetJogTimeTicks,
    csmSetJogDirRevKickTime,
    csmSetMotorOCPParams,
    csmSetMotorOcThresh,
    csm2_GotoPosition,
    csm2_SetSpeed,
    csmLEDTestMode, 
    csmStop,       
    #ifdef __MNET_DEVICE__
     #if !defined(CSM_QMTDC_163)&&!defined(CSM_DRAPE_MOTOR)   //Intended for CSM-2NmEx and CSM-SomfyEx that have External SPI flash
     ConsoleCmdTestFlash,
     #endif
    #endif    
    

#ifdef CSM2
    csm_testSlaveToBootloader,    
    csm_testSlaveExitBootloader,
    csmMotionTest,
    csm2GeneralQuery,
    csm2OvrConfig,
    csm2GetBldcData,
    csmEEPROM_Test,    
    csm2GetCurrData,
    csm2GetBuffer,
    csm2AutoCalibrate,
    csmGetJoinState,
    csmIsSlaveFWValid,
    csm2SetULTicks,   
    csm2EnableDAutocal,
    csm2GetBldcDebug,
    csm2_DC_OVR,
    csm2GetAvrgCurrData,
    csm2AdvAngleCal,
    csm2Reverse,
    csm2SetULimit,
    csm2SetLLimit,
    csm2Get24BusVoltage,
    csm2_i2cEeprom,
    csm2_resetSlave,
    csm2_getBadPktCount, 
    csm2_stopCalibration,
    csm2_prodSpeedMode,
    csm2_downMode,
    csm2MoveTicks,
    csm2TiltOpen,
    csm2TiltClose,
    csm2TiltEnable,
    csm2TiltPos,
    csm2TiltLimit,
    csm2OLAdvAngleCal,
    csm2DevStr,
    csm2getFlashCycles,
    csm2setSlowSpeedOverride,
    csm2_OLstartupPower,
    csm2_StopThresh,
    csm2_SetStopThreshDebug,
    drape_octh,
    csm2_Calibrate,
    csm2_ForceSlaveFW,
    csm2_MinRCB,
	csm2_GetCalibrationData,
    //CSM2_MYTEST,    
       
    #ifdef __CNET_DEVICE__ 
      csm2_suppressTimeBWRcb,
    #endif 

  #ifdef __MNET_DEVICE__
    restoreEx,
    nvInfoEx, 
    getModelVersion,
  #endif

  #ifdef __CNET_DEVICE__
       CmSyncPrintDebug,
  #endif
    
     
  #ifdef USE_MIDPOINT_CORRECTION  
    csm2_TrackingMode,    
    csm2_SetMidPointCorrection,
    csm2_SetMPC_Params,  
  #endif 
  
  #ifdef CSM_DRAPE_MOTOR
    drape_FindCloseLimit,
    drape_FindOpenLimit,
    drape_ShowLog,
    drape_ClearLog,
    drape_CaptureTouch,
    drape_TouchMode,
    drape_CaptureSpeed,
    drape_FindLimits,  
    drape_SetRef, 
    drape_setTouchSensitivity,
    drape_setTouch,
    drape_HighSpeed,
    drape_Debug,
  #endif
         
#endif
   #ifdef __CNET_DEVICE__ 
    #ifdef CSM2
    ShadeRcbLogConsoleCmd,
    ShadeRcbRptConsoleCmd,
    #endif
   #endif

};

/*! Make a table the supported commands */
static const UINT16 g_pnCmdFlags[] =
{
    0x0000,     // csmMassEraseEEPROM
    0x0000,     // csmShowUiState
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0000,
    0X0001,
    0X0001,
    0X0001,
    0X0001,
    0x0001,
    0x0001,    
    #ifdef __MNET_DEVICE__
     #if !defined(CSM_QMTDC_163)&&!defined(CSM_DRAPE_MOTOR)   //Intended for CSM-2NmEx and CSM-SomfyEx that have External SPI flash
     0x0001,
     #endif
    #endif  
    
    
#ifdef CSM2   
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0X0001,
    0x0001,
    0x0001,
    0X0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0X0001,
    0X0001,
    0X0001,    
    
    #ifdef __CNET_DEVICE__
     0x0001,
    #endif  

   #ifdef __MNET_DEVICE__
    0x0001,
    0x0001,
   #endif

   #ifdef __CNET_DEVICE__
    0x0001,
   #endif
           
  #ifdef USE_MIDPOINT_CORRECTION  
    0x0001,
    0x0001,
    0x0001,
  #endif  

  #ifdef CSM_DRAPE_MOTOR
    0x0001,
    0x0001,
    0X0001,
    0X0001,
    0X0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,    

  #endif
    
#endif

   #ifdef __CNET_DEVICE__
    #ifdef CSM2
    0x0001,
    0x0001,
    #endif
   #endif

};


/*-----------------4/16/2012 4:48PM-----------------
 * Return the number of application console entries
 * (0 if not used)
 * --------------------------------------------------*/
UINT8 AppCbConGetEntryCount(void)
{
    return (sizeof(g_pszcCmdNames) / sizeof(g_pszcCmdNames[0]));
}


/*-----------------4/16/2012 4:49PM-----------------
 * Return the command text for the selected command
 * entry
 * --------------------------------------------------*/
char const * AppCbConGetEntryCmdText( UINT8 index )
{
    return (index < AppCbConGetEntryCount()) ? g_pszcCmdNames[index] : "{null}";
}


/*-----------------4/16/2012 4:50PM-----------------
 * Return the command's help string text for the
 * selected command entry
 * --------------------------------------------------*/
char const * AppCbConGetEntryHelpText( UINT8 index )
{
    return (index < AppCbConGetEntryCount()) ? g_pszcCmdHelp[index] : "{null}";
}


/*-----------------4/16/2012 4:50PM-----------------
 * Copy the void/void function pointer for the
 * selected command
 * --------------------------------------------------*/
void (*AppCbConGetEntryFcnPtr(UINT8 index))(void)
{
    return (index < AppCbConGetEntryCount()) ? g_pCmdFunc[index] : NULL;
}


/*-----------------4/16/2012 4:52PM-----------------
 * Return a 16b word with the command's debug level
 * and privilege level.  The later is not defined
 * yet for Cresnet
 * --------------------------------------------------*/
UINT16 AppCbConGetEntryFlags( UINT8 index )
{
    return (index < AppCbConGetEntryCount()) ? g_pnCmdFlags[index] : 0x0000;
}


/**
 * \brief   A misc status/event notifcation callback transport operations
 * \details Applications in general should not need to do anything with this callback but may be able to enhance operation
 *          or debugging capability by checking the status values; this is because this callback will include status codes
 *          that are specific to a particular transport implementation
 * \return  void
 * \param   status A CmStatusEventErrorCodes with a status code for an RF link/transport event
 * \param   parameter A UINT16 with optional parameter data associated with the event.
 */
void CmTransCbMiscStatusNotify( CmStatusEventErrorCodes status, UINT16 parameter )
{
}


/*-----------------10/1/2012 11:45AM----------------
 * The lp-client version of getting a console entry
 * Hook into existing implementation
 * --------------------------------------------------*/
#ifdef __MNET_DEVICE__
void LpAppCbConsoleCopyEntry( UINT8 index, COMMAND_STRUCTURE * psCmdStructEntry )
{
    psCmdStructEntry->m_pCmdText = AppCbConGetEntryCmdText(index);
    psCmdStructEntry->m_pCmdHelp = AppCbConGetEntryHelpText(index);
    psCmdStructEntry->m_pFunc = AppCbConGetEntryFcnPtr(index);
    psCmdStructEntry->nDebugLevel = AppCbConGetEntryFlags(index);  // copy the flags as is for now
    psCmdStructEntry->nPrvldgLevel = 0;
}


/*-----------------10/1/2012 11:54AM----------------
 * Return the number of entries in app console section
 * --------------------------------------------------*/
UINT8 LpAppCbConsoleGetEntryCount(void)
{
    return AppCbConGetEntryCount();
}

#endif


/**
 * \brief   Callback, console print func, asks for any extra ver info to report and print on console
 * \details This is called from the console cmd processing for the ver -v  command.
 * \return  void
 * \param   void
 */
void console_CbPrintExtraVersionInfo(void)
{
    //not supported
}


/*-----------------6/17/2016 10:25AM----------------
 * this callback is for testing RCB output operation
 * it is called from the event_cresnet_tx event
 * where RCB data is actually submitted to the
 * tx buffering
 * --------------------------------------------------*/
void AppCbOnRcbSendAttempt( UINT8 slot, UINT16 channel, UINT16 level, UINT32 ramptime )
{
     UINT16 level16b = ShdLimGetLevelFromPosition(
        EEPROM_Sys.sys_nUpperLimit,
        EEPROM_Sys.sys_nLowerLimit,
        ctrl_GetNowPosition()
    );

    // we only have one channel so skip the slot/channel checking for now
    // call the RCB logging with the necessary data
    //ShadeRcbLogOnSentAttempt( level, ramptime, ShdPctrlGetAvgSpeed() );
    #ifdef __CNET_DEVICE__
      ShadeRcbLogOnSentAttempt( level, ramptime, ShdPctrlGetAvgRpmMagX100(), level16b );
    #endif
}

