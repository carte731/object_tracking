/************************************************************************
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
** File: object_tracker_msgids.h
**
** Purpose: 
**  Define Object Tracking App  Message IDs
**
** Notes:
**
**
*************************************************************************/
#ifndef _object_tracker_msgids_h_
#define _object_tracker_msgids_h_

#define OBJECT_TRACKER_CMD_MID            0x1972
#define OBJECT_TRACKER_SEND_HK_MID        0x1973
#define OBJECT_TRACKER_HK_TLM_MID		  0x0973

// Object Tracker Switch MSG-ID
#define OBJECT_TRACKER_SWITCH_MID		  0x1970

// Object Tracker publish essentials state MSG-ID
#define OBJECT_TRACKER_ESS_STATE_PUB		  0x1974

// Object Tracker request command for complete object history
#define OBJECT_TRACKER_COM_STATE_REQ		  0x1975

// Object Tracker publish complete state MSG-ID
#define OBJECT_TRACKER_COM_STATE_PUB		  0x1976

#endif /* _object_tracker_msgids_h_ */

/************************/
/*  End of File Comment */
/************************/
