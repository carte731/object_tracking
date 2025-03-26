/*******************************************************************************
**
**      GSC-18128-1, "Core Flight Executive Version 6.7"
**
**      Copyright (c) 2006-2019 United States Government as represented by
**      the Administrator of the National Aeronautics and Space Administration.
**      All Rights Reserved.
**
**      Licensed under the Apache License, Version 2.0 (the "License");
**      you may not use this file except in compliance with the License.
**      You may obtain a copy of the License at
**
**        http://www.apache.org/licenses/LICENSE-2.0
**
**      Unless required by applicable law or agreed to in writing, software
**      distributed under the License is distributed on an "AS IS" BASIS,
**      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
**      See the License for the specific language governing permissions and
**      limitations under the License.
**
** File: object_tracker.h
**
** Purpose:
**   This file is main hdr file for the TALKER application.
**
**
*******************************************************************************/

#ifndef _object_tracker_h_
#define _object_tracker_h_

/*
** Required header files.
*/
#include "cfe.h"
#include "cfe_error.h"
#include "cfe_evs.h"
#include "cfe_sb.h"
#include "cfe_es.h"

#include "object_tracker_msgids.h"
#include "object_tracker_msg.h"

// Sync Node Library functions
// Decodes ROS messages into structs using cJSON
// Needed for Object_tracking to work properly
#include "sync_node_lib.h"


/***********************************************************************/
// DEBUG - 3
#define OBJECT_PIPE_DEPTH                     128 /* Depth of the Command Pipe for Application */

// The fileIO system location for ROS2-JSON msgs
#define ROS2_FILE_LOC                         "/root/img_data/" 
#define BACKUP_HISTORY                        false;

/************************************************************************
** Type Definitions
*************************************************************************/

/*
 * Buffer to hold telemetry data prior to sending
 * Defined as a union to ensure proper alignment for a CFE_SB_Msg_t type
 */
typedef union
{
    CFE_SB_Msg_t        MsgHdr;
    OBJECT_HkTlm_t      HkTlm;
} OBJECT_HkBuffer_t;

/*
** Global Data
*/
typedef struct
{
    /*
    ** Command interface counters...
    */
    uint8                 CmdCounter;
    uint8                 ErrCounter;

    /*
    ** Housekeeping telemetry packet...
    */
    OBJECT_HkBuffer_t     HkBuf;

    // Used for tracking objects based on 'class_id' and corresponding index position
    Object_Master_List_t  object_track_listing;

    // Outbound essential object state message
    OBJECT_State_t object_essential_state_msg;

    // Outbound complete object state history message
    // May switch to normal array - not sure if
    // zero copy will work with linked lists
    OBJECT_history_t *object_complete_state_msg;

    /*
    ** Run Status variable used in the main processing loop
    */
    uint32 RunStatus;

    /*
    ** Operational data (not reported in housekeeping)...
    */
    CFE_SB_PipeId_t    CommandPipe;
    CFE_SB_MsgPtr_t    MsgPtr;

    /*
    ** Initialization data (not reported in housekeeping)...
    */
    char     PipeName[16];
    uint16   PipeDepth;
} OBJECT_TrackerData_t;

/****************************************************************************/
/*
** Local function prototypes.
**
** Note: Except for the entry point (OBJECT_TrackerMain), these
**       functions are not called from any other source module.
*/
void  OBJECT_TrackerMain(void);
int32 OBJECT_TrackerInit(void);
void  OBJECT_ProcessCommandPacket(CFE_SB_MsgPtr_t Msg);
void  OBJECT_ProcessGroundCommand(CFE_SB_MsgPtr_t Msg);
int32 OBJECT_ReportHousekeeping(const CCSDS_CommandPacket_t *Msg);
int32 OBJECT_ResetCounters(const OBJECT_ResetCounters_t *Msg);
int32 OBJECT_Process(const OBJECT_Process_t *Msg);
int32 OBJECT_Noop(const OBJECT_Noop_t *Msg);
void  OBJECT_GetCrc(const char *TableName);
bool  OBJECT_VerifyCmdLength(CFE_SB_MsgPtr_t Msg, uint16 ExpectedLength);

// Object specific functions
void OBJECT_Save_States();
void OBJECT_Switch_Request( const OBJECT_Switch_t *Msg );
void OBJECT_Publish_States();
void OBJECT_Publish_Complete_States();

#endif /* _object_tracker_h_ */
