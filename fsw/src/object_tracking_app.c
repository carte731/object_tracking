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
** File: object_tracker.c
**
** Purpose:
**   This file contains the source code for a talker application that does
**   nothing and is the minimum required for a valid cFS application.
**
*******************************************************************************/

/*
** Include Files:
*/
#include "object_tracker_events.h"
#include "object_tracker_version.h"
#include "object_tracker.h"

/*
** global data
*/
OBJECT_TrackerData_t OBJECT_TrackerData;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/* OBJECT_TrackerMain() -- Application entry point and main process loop        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void OBJECT_TrackerMain( void )
{
    int32  status;

    /*
    ** Register the app with Executive services
    */
    CFE_ES_RegisterApp();

    /*
    ** Perform application specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    status = OBJECT_TrackerInit();
    if (status != CFE_SUCCESS)
    {
        OBJECT_TrackerData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    /*
    ** TALKER Runloop
    */
    while (CFE_ES_RunLoop(&OBJECT_TrackerData.RunStatus) == true)
    {

        // Pulls the newest data from ROS-Msg files and saves it to CMD struct 
        OBJECT_Save_States();

        // Publishes the newest enabled objects to the software bus 
        OBJECT_Publish_States();

        status = CFE_SB_RcvMsg(&OBJECT_TrackerData.MsgPtr,
                               OBJECT_TrackerData.CommandPipe,
                               500);


        if (status == CFE_SUCCESS)
        {
            OBJECT_ProcessCommandPacket(OBJECT_TrackerData.MsgPtr);
        }

    }

    CFE_ES_ExitApp(OBJECT_TrackerData.RunStatus);

} /* End of OBJECT_TrackerMain() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* OBJECT_TrackerInit() --  initialization                                      */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 OBJECT_TrackerInit( void )
{

    int32    status;

    OBJECT_TrackerData.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
    ** Initialize app command execution counters
    */
    OBJECT_TrackerData.CmdCounter = 0;
    OBJECT_TrackerData.ErrCounter = 0;

    // File counter
    OBJECT_TrackerData.fileItr = 0;

    /*
    ** Initialize app configuration data
    */
    OBJECT_TrackerData.PipeDepth = OBJECT_PIPE_DEPTH;

    strcpy(OBJECT_TrackerData.PipeName, "OBJECT_CMD_PIPE");

    /*
    ** Register the events
    */
    if ((status = CFE_EVS_Register(NULL, 0, 0)) != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Talker App: Error Registering Events, RC = %lu\n",
                             (unsigned long)status);
        return ( status );
    }

    /*
    ** Initialize housekeeping packet (clear user data area).
    */
    CFE_SB_InitMsg(&OBJECT_TrackerData.HkBuf.MsgHdr,
                   OBJECT_TRACKER_HK_TLM_MID,
                   sizeof(OBJECT_TrackerData.HkBuf),
                   true);     
                   
    // Used for outbound essentials object state messages
    CFE_SB_InitMsg(&OBJECT_TrackerData.object_essential_state_msg.MsgHdr,
                    OBJECT_TRACKER_ESS_STATE_MID,
                    sizeof(OBJECT_TrackerData.object_essential_state_msg),
                    true);   

    // Used for outbound complete object state messages
    CFE_SB_InitMsg(&OBJECT_TrackerData.object_complete_state_msg.MsgHdr,
                    OBJECT_TRACKER_COM_STATE_MID,
                    sizeof(OBJECT_TrackerData.object_complete_state_msg),
                    true);   

    /*
    ** Create Software Bus message pipe.
    */
    status = CFE_SB_CreatePipe(&OBJECT_TrackerData.CommandPipe,
                               OBJECT_TrackerData.PipeDepth,
                               OBJECT_TrackerData.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Talker App: Error creating pipe, RC = 0x%08lX\n",
                             (unsigned long)status);
        return ( status );
    }

    /*
    ** Subscribe to Housekeeping request commands
    */
    status = CFE_SB_Subscribe(OBJECT_TRACKER_SEND_HK_MID,
                              OBJECT_TrackerData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Talker App: Error Subscribing to HK request, RC = 0x%08lX\n",
                             (unsigned long)status);
        return ( status );
    }

    /*
    ** Subscribe to ground command packets
    */
    status = CFE_SB_Subscribe(OBJECT_TRACKER_CMD_MID,
                              OBJECT_TrackerData.CommandPipe);
    if (status != CFE_SUCCESS )
    {
        CFE_ES_WriteToSysLog("Talker App: Error Subscribing to Command, RC = 0x%08lX\n",
                             (unsigned long)status);

        return ( status );
    }

    // Subscribing to direct requests for object tracking enable switch
    status = CFE_SB_Subscribe(OBJECT_TRACKER_SWITCH_MID,
        OBJECT_TrackerData.CommandPipe);
    if (status != CFE_SUCCESS )
    {
        CFE_ES_WriteToSysLog("Talker App: Error Subscribing to Enable/Disable Commands, RC = 0x%08lX\n",
            (unsigned long)status);

        return ( status );
    }



    CFE_EVS_SendEvent (OBJECT_STARTUP_INF_EID,
                       CFE_EVS_EventType_INFORMATION,
                       "TALKER App Initialized. Version %d.%d.%d.%d",
                       OBJECT_TRACKER_MAJOR_VERSION,
                       OBJECT_TRACKER_MINOR_VERSION,
                       OBJECT_TRACKER_REVISION,
                       OBJECT_TRACKER_MISSION_REV);

    return ( CFE_SUCCESS );

} /* End of OBJECT_TrackerInit() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  OBJECT_ProcessCommandPacket                                        */
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the OBJECT    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void OBJECT_ProcessCommandPacket( CFE_SB_MsgPtr_t Msg )
{
    CFE_SB_MsgId_t  MsgId;

    MsgId = CFE_SB_GetMsgId(Msg);

    switch (MsgId)
    {
        case OBJECT_TRACKER_CMD_MID:
            OBJECT_ProcessGroundCommand(Msg);
            break;

        case OBJECT_TRACKER_SEND_HK_MID:
            OBJECT_ReportHousekeeping((CCSDS_CommandPacket_t *)Msg);
            break;

        // Allows ground GCS to change publish mode for each tracked object
        case OBJECT_TRACKER_SWITCH_CC:
            // Passing uniform message payload (same payload for ground (GCS) and app commands/messages)
            OBJECT_Switch_Request((OBJECT_Switch_t *) Msg->payload);
            break; 

        // Allows ground GCS to change publish mode for each tracked object
        case OBJECT_TRACKER_COMPLETE_CC:
            // Passing uniform message payload (same payload for ground (GCS) and app commands/messages)
            OBJECT_Publish_Complete_State((OBJECT_Switch_COM_App_t *) Msg);
            break;

        default:
            CFE_EVS_SendEvent(OBJECT_INVALID_MSGID_ERR_EID,
                            CFE_EVS_EventType_ERROR,
                            "TALKER: invalid command packet,MID = 0x%x",
                            MsgId);
            break;
    }

    return;

} /* End OBJECT_ProcessCommandPacket */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* OBJECT_ProcessGroundCommand() -- TALKER ground commands                */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void OBJECT_ProcessGroundCommand( CFE_SB_MsgPtr_t Msg )
{
    uint16 CommandCode;

    CommandCode = CFE_SB_GetCmdCode(Msg);

    /*
    ** Process "known" TALKER app ground commands
    */
    switch (CommandCode)
    {
        case OBJECT_TRACKER_NOOP_CC:
            if (OBJECT_VerifyCmdLength(Msg, sizeof(OBJECT_Noop_t)))
            {
                OBJECT_Noop((OBJECT_Noop_t *)Msg);
            }

            break;

        case OBJECT_TRACKER_RESET_COUNTERS_CC:
            if (OBJECT_VerifyCmdLength(Msg, sizeof(OBJECT_ResetCounters_t)))
            {
                OBJECT_ResetCounters((OBJECT_ResetCounters_t *)Msg);
            }

            break;

        case OBJECT_TRACKER_PROCESS_CC:
            if (OBJECT_VerifyCmdLength(Msg, sizeof(OBJECT_Process_t)))
            {
                OBJECT_Process((OBJECT_Process_t *)Msg);
            }

            break;  
            
        case OBJECT_TRACKER_SWITCH_MID:
            if (OBJECT_VerifyCmdLength(Msg, sizeof(OBJECT_Switch_GCS_Header_t)))
            {
                // Passing uniform message payload (same payload for ground (GCS) and app commands/messages)
                OBJECT_Switch_Request((OBJECT_Switch_t *) Msg->payload);
            } 

            break;   

        case OBJECT_TRACKER_COM_STATE_REQ:
            if (OBJECT_VerifyCmdLength(Msg, sizeof(OBJECT_Switch_COM_GCS_t)))
            {
                // Passing uniform message payload (same payload for ground (GCS) and app commands/messages)
                OBJECT_Publish_Complete_State((OBJECT_Switch_COM_GCS_t *) Msg);
            } 

            break;

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(OBJECT_COMMAND_ERR_EID,
                              CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d",
                              CommandCode);
            break;
    }

    return;

} /* End of OBJECT_ProcessGroundCommand() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  OBJECT_ReportHousekeeping                                        */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 OBJECT_ReportHousekeeping( const CCSDS_CommandPacket_t *Msg )
{
    /*
    ** Get command execution counters...
    */
    OBJECT_TrackerData.HkBuf.HkTlm.Payload.CommandErrorCounter = OBJECT_TrackerData.ErrCounter;
    OBJECT_TrackerData.HkBuf.HkTlm.Payload.CommandCounter = OBJECT_TrackerData.CmdCounter;

    /*
    ** Send housekeeping telemetry packet...
    */
    CFE_SB_TimeStampMsg(&OBJECT_TrackerData.HkBuf.MsgHdr);
    CFE_SB_SendMsg(&OBJECT_TrackerData.HkBuf.MsgHdr);

    return CFE_SUCCESS;

} /* End of OBJECT_ReportHousekeeping() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* OBJECT_Noop -- TALKER NOOP commands                                    */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 OBJECT_Noop( const OBJECT_Noop_t *Msg )
{

    OBJECT_TrackerData.CmdCounter++;

    CFE_EVS_SendEvent(OBJECT_COMMANDNOP_INF_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "TALKER: NOOP command  Version %d.%d.%d.%d",
                      OBJECT_TRACKER_MAJOR_VERSION,
                      OBJECT_TRACKER_MINOR_VERSION,
                      OBJECT_TRACKER_REVISION,
                      OBJECT_TRACKER_MISSION_REV);

    return CFE_SUCCESS;

} /* End of OBJECT_Noop */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  OBJECT_ResetCounters                                             */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 OBJECT_ResetCounters( const OBJECT_ResetCounters_t *Msg )
{

    OBJECT_TrackerData.CmdCounter = 0;
    OBJECT_TrackerData.ErrCounter = 0;

    CFE_EVS_SendEvent(OBJECT_COMMANDRST_INF_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "TALKER: RESET command");

    return CFE_SUCCESS;

} /* End of OBJECT_ResetCounters() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* OBJECT_VerifyCmdLength() -- Verify command packet length                 */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool OBJECT_VerifyCmdLength( CFE_SB_MsgPtr_t Msg, uint16 ExpectedLength )
{
    bool result = true;

    uint16 ActualLength = CFE_SB_GetTotalMsgLength(Msg);

    /*
    ** Verify the command packet length.
    */
    if (ExpectedLength != ActualLength)
    {
        CFE_SB_MsgId_t MessageID   = CFE_SB_GetMsgId(Msg);
        uint16         CommandCode = CFE_SB_GetCmdCode(Msg);

        CFE_EVS_SendEvent(OBJECT_LEN_ERR_EID,
                          CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %d, Len = %d, Expected = %d",
                          MessageID,
                          CommandCode,
                          ActualLength,
                          ExpectedLength);

        result = false;

        OBJECT_TrackerData.ErrCounter++;
    }

    return( result );

} /* End of OBJECT_VerifyCmdLength() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  OBJECT_Process                                                     */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function Process Generalized Ground Station Commands          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void OBJECT_Process(const OBJECT_Process_t *Msg){

    return CFE_SUCCESS;

} /* End of OBJECT_Save_States */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  OBJECT_Save_States                                                 */
/*                                                                            */
/*  Purpose:                                                                  */
/*         Saves object states to app CMD struct                              */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
// TO-DO: USE C BASED GENERICS AND MACROS TO SUPPORT MULTIPLE DATA/STRUCT TYPES
void OBJECT_Save_States(){
    // TO-DO: Turn this into a GENERIC AND MARCO TO GENERALIZE ARRAY TYPE
    // Creates a rover array to hold entries from the YOLO-ROS JSON file
    rover_array rovers;

    // TO-DO: Replace with a function that scans directory contents and processes all of them
    //        into linked-list. Or just take the most recent input.
    // Updates the file path with the file number 
    char fileLoc[100];
    sprintf(fileLoc, "%sOutput_%d", ROS2_FILE_LOC, OBJECT_TrackerData.fileItr++);

    // Loads object array with data from JSON file 
    sync_fusion_injest(&rovers, fileLoc);

    // Checks if it's too large and cleans out the Linked list first with clean-up function.
    // Will save the data to software bus if flag (BACKUP_HISTORY) is true.
    if(BACKUP_HISTORY){
        // TO-DO: If object linked-list is too long,
        // function loops over entries and checks length.
        // publishes buffer space to software bus, which clears out
        // the space before adding new the new entry. 
        OBJECT_Publish_Complete_State_GEN();
    }

    // Saves 'rover_array' elemets to local CMD structs
    // Inserts the elements in order based on time stamps (sec and nano sec)
    amortizedInsert(rover_array *rovers);

} /* End of OBJECT_Save_States */

// TO-DO: USE C BASED GENERICS AND MACROS TO SUPPORT MULTIPLE DATA/STRUCT TYPES
// TO-DO: BREAK APART INTO MULTIPLE SUB-FUNCTIONS - TOO LONG

// Since most new entries (theoretically) should be latest and will
// be added to the end of the linked-list (O(1)). Some entries may have
// been delayed, meaning it will require a linear search and insertion (O(n)).
// Making this an amortized linked list insert.
int32 amortizedInsert(rover_array *rovers){
    rover_state travelerNode;
    Object_Master_Node_t *headNode;

    // Array timestamps for the YOLO capture
    uint32 time_sec = rovers->timeStamp_sec;
    uint32 time_nanoSec = rovers->timeStamp_nanoSec;

    // Used for tracking each elements time for sorting
    uint32 local_time_sec;
    uint32 local_time_nanoSec;

    //// Front Loading Loop - If master node is empty - add first element to linked list of objects
    //// Collects elements for Appending list.

    // Array length for the YOLO-ROS JSON
    int arrayLen = rovers->arrayLen;

    // Creating an array for inserting in the linked list
    rover_array objectAppendStack[rovers->arrayLen];

    // Used for tracking future insertions
    int stackAppendSize = 0;

    int itr;
    for(itr = 0; itr < arrayLen; itr++){
        // Setting comparison node
        travelerNode = rovers->rovers_array[itr];

        // Grabbing the class-id, this dictates the array index position
        uint8 indx = travelerNode.class_id;

        // Mounting the head node in the linked list
        headNode = OBJECT_TrackerData.object_track_listing.object_list[indx];

        // If the master node doesn't exist - create it and add the first element
        if(headNode == NULL){
            Object_Master_Node_t new_master_node;
            new_master_node.class_id = indx;
            headNode = &new_master_node;
        }
    
        // If there is no starter node - add it
        if(headNode->start_node == NULL){
            headNode->start_node = travelerNode;
            headNode->latest_node = travelerNode;
            headNode->total_node_len++;
        } else {
            // Add it to the append stack to append in next phase
            objectAppendStack[stackAppendSize++] = travelerNode;
        }
    }


    //// Appending loop - will check and add the elements to the end of the linked list
    //// Also collects insert elements for insertion loop

    // Array length for the YOLO-ROS JSON
    //arrayLen = rovers->arrayLen;

    // Creating an array for inserting in the linked list
    rover_array objectInsertStack[stackAppendSize];

    // Used for tracking future insertions
    int stackinsertSize = 0;
    
    for(itr = 0; itr < stackAppendSize; itr++){
        // Setting comparison node
        //travelerNode = rovers->rovers_array[itr];
        travelerNode = objectAppendStack[itr];

        // Grabbing the class-id, this dictates the array index position
        uint8 indx = travelerNode.class_id;

        // Mounting the head node in the linked list
        headNode = OBJECT_TrackerData.object_track_listing.object_list[indx];

        // Checking if the latest timestamp is less than the new input
        if(headNode->enable_switch){

            // Pulling the local time from the latest node
            local_time_sec = headNode->latest_node->latest_node.object_state.timeStamp_sec;
            local_time_nanoSec = headNode->latest_node->latest_node.object_state.timeStamp_nanoSec;

            if((local_time_sec >= time_sec) && (local_time_nanoSec >= time_nanoSec)){
                // Add to the end of the linked list
                appendList(headNode, travelerNode);
            } else {
                // Add to the stack to loop over later
                objectInsertArr[stackinsertSize++] = travelerNode;
            }
        }
    }


    //// Insertion Loop - will insert element in the proper position, won't publish until entire list is sent to software bus 
    //// Nothing to insert, exit from function
    if(stackinsertSize <= 0){
        return(2);
    }

    // Brute Force method for inserting in linear O(n * m) == O(n) time
    // A way of de-nesting the loop needed for linked list insertion

    // Setting comparison node
    rover_state insertTraveler = objectInsertArr[itr];

    // Grabbing the class-id, this dictates the array index position
    uint8 indx = travelerNode.class_id;

    // Mounting the staring node
    Object_Node_t *travelerObjectNode = OBJECT_TrackerData.object_track_listing.object_list[indx]->start_node;


    itr = 0;
    int outer_itr = 0;
    while(itr < stackinsertSize){
        // Pulling the local time from the latest node
        local_time_sec = travelerObjectNode.object_state.timeStamp_sec;
        local_time_nanoSec = travelerObjectNode.object_state.timeStamp_nanoSec;

        // This only activates when bounds are present, should never activate unless there's an error
        if(travelerObjectNode == NULL){
            appendList(headNode, insertTraveler);
        }

        if((local_time_sec >= time_sec) && (local_time_nanoSec >= time_nanoSec)){
            // Add to the end of the linked list
            insert(stackinsertSize, insertTraveler);

            // Preincrement and load the next object node
            insertTraveler = objectInsertArr[++itr];

            // Grabbing the class-id, this dictates the array index position
            indx = insertTraveler.class_id;
        
            // Mounting the next staring node
            travelerObjectNode = OBJECT_TrackerData.object_track_listing.object_list[indx]->start_node;
        } else {
            // Increment the linked list to check the next node
            travelerObjectNode = travelerObjectNode.next_node;
        }
        
    }

    // Done
    return(1);

}

// TO-DO: USE C BASED GENERICS AND MACROS TO SUPPORT MULTIPLE DATA/STRUCT TYPES
void insertLL(Object_Node_t *objectNode, rover_state *objectData){
    // Creating new object to insert
    Object_Node_t newObject;

    // Setting the object data
    newObject.object_state = objectData;

    // Changing what the next objects previous object link
    objectNode->next_node->previous_node = &newObject;

    // Setting the new inserted objects next and previous nodes
    // The new nodes will always be place in front of the previous node
    newObject.previous_node = objectNode;
    newObject.next_node = objectNode->next_node;

    // The new node is placed in front of the old node
    // Changing association 
    objectNode->next_node = &newObject;

}

// TO-DO: USE C BASED GENERICS AND MACROS TO SUPPORT MULTIPLE DATA/STRUCT TYPES
void appendLL(Object_Master_Node_t *headObject, rover_state *newNode){
    // Initializing a traveler node for the linked list
    Object_Node_t newObject;

    // Copying the object over
    // TO-DO: USE C BASED GENERICS AND MACROS TO SUPPORT MULTIPLE DATA/STRUCT TYPES
    newObject.object_state = newNode;

    // Link the nodes
    newObject.previous_node = headObject->latest_node;
    newObject.next_node = NULL;

    // Initialize the publish bool
    newObject.beenPublished = false;

    // Relinking the master node for the object
    headObject->latest_node->next_node = &newObject;

    // Relinking master node to new apended node
    headObject->latest_node = &newObject;

    // Increasing the total node len size
    newObject->total_node_len++;

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  OBJECT_Switch_Request                                              */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function Process Ground Station and app switch requests       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void  OBJECT_Switch_Request(const OBJECT_Switch_t *Msg){

    // Copying over the variable values
    // The class_id corresponds to the array index, allowing for basic hashing O(1)
    uint8 class_id_indx = Msg->class_id;

    // Saving the message bool value to enable or disable publishing to the software bus
    bool enable_disable = Msg->switchCMD;

    // Switching the publisher flag to 'on' or 'off' in the head element in the array
    OBJECT_TrackerData.object_track_listing.object_list[class_id_indx].enable_switch = enable_disable;

} /* End of OBJECT_Switch_Request */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  OBJECT_Publish_States                                              */
/*                                                                            */
/*  Purpose:                                                                  */
/*         Publishes object states to software-bus                            */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void OBJECT_Publish_States(){

    // Publish Essentials message to software bus if enabled
    CFE_SB_TimeStampMsg(&OBJECT_TrackerData.HkBuf.MsgHdr);
    CFE_SB_SendMsg(&OBJECT_TrackerData.HkBuf.MsgHdr);

} /* End of OBJECT_Publish_States */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  OBJECT_Publish_Complete_States                                     */
/*                                                                            */
/*  Purpose:                                                                  */
/*         Publishes object complete history to software-bus                  */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
// TO-DO: USE C BASED GENERICS AND MACROS TO SUPPORT MULTIPLE DATA/STRUCT TYPES
// TO-DO: Will implement in the future
void OBJECT_Publish_Complete_State(const OBJECT_Switch_t *Msg){
    
    // Publish Complete history message using 'zero_copy' by using CFE_SB_TransmitBuffer(SAMPLE_AppData.BigPktBuf, BufferHandle, true);
    // SAMPLE_AppData.BigPktBuf = (SAMPLE_BigPkt_Buffer_t *)CFE_SB_AllocateMessageBuffer(sizeof(SAMPLE_BigPkt_t));
    // Only publish enabled elements in master array

} /* End of OBJECT_Publish_States */

// TO-DO: If object linked-list is too long,
// function loops over entries and checks length.
// publishes buffer space to software bus, which clears out
// the space before adding new the new entry.
// TO-DO: Will implement in the future 
void OBJECT_Publish_Complete_State_GEN(){
}