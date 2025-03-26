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
** File: object_tracker_msg.h
**
** Purpose:
**  Define TALKER App  Messages and info
**
** Notes:
**
**
*******************************************************************************/
#ifndef _object_tracker_msg_h_
#define _object_tracker_msg_h_

// Maximum amount of objects to track.
// The index position matches the 'class_id', this allows for O(1) access due to basic hashing
#define MAX_OJBECT_TRACKING                   10  
#define MAX_LINKED_LIST_LEN                   50

/*
** TALKER App command codes
*/
#define OBJECT_TRACKER_NOOP_CC                 0
#define OBJECT_TRACKER_RESET_COUNTERS_CC       1
#define OBJECT_TRACKER_PROCESS_CC              2
#define OBJECT_TRACKER_SWITCH_CC               3
#define OBJECT_TRACKER_COMPLETE_CC             4

/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct
{
   uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];

} OBJECT_NoArgsCmd_t;

/*
** The following commands all share the "NoArgs" format
**
** They are each given their own type name matching the command name, which_open_mode
** allows them to change independently in the future without changing the prototype
** of the handler function
*/
typedef OBJECT_NoArgsCmd_t      OBJECT_Noop_t;
typedef OBJECT_NoArgsCmd_t      OBJECT_ResetCounters_t;
typedef OBJECT_NoArgsCmd_t      OBJECT_Process_t;


/*************************************************************************/
/*
** Type definition (TALKER App housekeeping)
*/

typedef struct
{
    uint8              CommandErrorCounter;
    uint8              CommandCounter;
    uint8              spare[2];
} OBJECT_HkTlm_Payload_t;

typedef struct
{
    uint8              TlmHeader[CFE_SB_TLM_HDR_SIZE];
    OBJECT_HkTlm_Payload_t  Payload;

} OS_PACK OBJECT_HkTlm_t;

// DEBUG - 3
/*************************************************************************/
/*
** Type definition (TALKER App telemetry CMD counter)
*/

typedef struct
{
    char*              TeleMSG;
    uint8              spare[2];
} OBJECT_HkTlm_Counter_Payload_t;

typedef struct
{
    uint8              TlmHeader[CFE_SB_TLM_HDR_SIZE];
    OBJECT_HkTlm_Counter_Payload_t  Payload;

} OS_PACK OBJECT_HkTlm_Counter_t;

typedef union
{
    CFE_SB_Msg_t                MsgHdr;
    OBJECT_HkTlm_Counter_t      CounterTlm;
} OBJECT_CounterBuffer_t;

// DEBUG - 3
/*************************************************************************/
/*
** Type definition (TALKER App counter)
*/



// Contains the essential data for COSMOS
typedef struct
{
    uint32      timeStamp_sec;
    uint32      timeStamp_nanoSec;
    uint8       class_id;
    char        class_name[10];
    double      confidenceScore;
    char        object_id[10];
    double      distance;
    double      orientation;
    // Maybe keypoints-2D as well
    //keypoint3D   keypoint_3D_listing[10];
} OBJECT_Essentials_Data_t;

// Object Tracking essentials message 
typedef struct
{
    CFE_SB_Msg_t                MsgHdr;
    OBJECT_Essentials_Data_t    payload;
} OBJECT_State_t;



// Message contains header node
// This allows for complete traversal of the object historys movement
typedef struct
{
    // The head node in the object linked list history
    // The user can iterate over the nodes for the full object detection history
    Object_Master_Node_t object_master_node;

} OBJECT_Complete_Data_t;

// Object-Tracking complete history
// Sends the buffer pointer over, this prevents copying over
typedef struct
{
    CFE_SB_Buffer_t           objectSBBuf;
    OBJECT_Complete_Data_t    payload;
} OBJECT_history_App_Header_t;



// TO-DO: USE GENERICS AND MACROS FOR UNIFORM MESSAGE HEADER
// Used for requesting the publication of an object's state on the software bus.
typedef struct
{
    // The ojects annotation id number
    uint8       class_id;

    // Switch state to turn 'on' or 'off' the 'enable_switch' CMD struct variable
    bool        switchCMD;

} OBJECT_Switch_t;

// Switch message app-to-app header
typedef struct
{
    CFE_SB_Msg_t                MsgHdr;
    OBJECT_switch_t             payload;
} OBJECT_Switch_App_Header_t;

// Switch message ground-to-app header
typedef struct
{
    uint8                       CmdHeader[CFE_SB_CMD_HDR_SIZE];
    OBJECT_switch_t             payload;
} OBJECT_Switch_GCS_Header_t;



// TO-DO: USE GENERICS AND MACROS FOR UNIFORM MESSAGE HEADER
// Used for requesting the COMPLETE publication hsitory of an object's state on the software bus.
typedef struct
{
    // The ojects annotation id number
    uint8       class_id;

    // Switch state to turn 'on' or 'off' the 'enable_switch' CMD struct variable
    bool        switchCMD;

} OBJECT_Switch_t;

// Switch message app-to-app header
typedef struct
{
    CFE_SB_Msg_t                MsgHdr;
    uint8                       class_id;
} OBJECT_Switch_COM_App_t;

// Switch message ground-to-app header
typedef struct
{
    uint8                       CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint8                       class_id;
} OBJECT_Switch_COM_GCS_t;



// Tracks all the ojects in YOLO model, each element contains a linked list with a history of sightings 
// The index position matches the 'class_id', this allows for O(1) access due to basic hashing
typedef union
{
    // Array used for tracking YOLO object detections 
    Object_Master_Node_t object_list[MAX_OJBECT_TRACKING];

} Object_Master_List_t;

// Object Tracking header node
typedef struct
{
    // The ojects annotation id number
    uint8       class_id;

    // Dictates which object states will be sent on the software bus 
    bool enable_switch;

    // The starting and latest observations - this allows for trversal of object history
    object_node_t start_node;
    object_node_t latest_node;

    // Total length of the linked-list
    // Used for easier iterating and maxed linked list checking
    int total_node_len;

} Object_Master_Node_t;




// Object-node - holds the rover data and neighbor nodes
// TO-DO: Will make into MACRO later so it can hold multiple types of data and cJSON structs.
typedef struct 
{
    // The raw saved data from cJSON ROS2-YOLO file 
    rover_state object_state;
    
    // The neighboring object state nodes
    Object_Node_t previous_node;
    Object_Node_t next_node;

    // Tracks if it has been published on the software bus yet
    bool          beenPublished;
    
} Object_Node_t;


#endif /* _object_tracker_msg_h_ */

/************************/
/*  End of File Comment */
/************************/
