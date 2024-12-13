; Copyright (c) 2024 Carologistics
;
; Licensed under the Apache License, Version 2.0 (the "License");
; you may not use this file except in compliance with the License.
; You may obtain a copy of the License at
;
;     http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS,
; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.

(deftemplate game-state
  (slot state (type SYMBOL) (allowed-values INIT WAIT-START RUNNING PAUSED))
  (slot phase (type SYMBOL) (allowed-values PRE_GAME SETUP EXPLORATION PRODUCTION POST_GAME))
  (slot points (type INTEGER))
  (slot points-other (type INTEGER))
  (slot team (type STRING))
  (slot team-other (type STRING))
  (slot team-color (type SYMBOL) (allowed-values NOT-SET CYAN MAGENTA) (default NOT-SET))
  (slot field-width (type INTEGER))
  (slot field-height (type INTEGER))
  (slot field-mirrored (type SYMBOL) (allowed-values FALSE TRUE))
)

(deftemplate machine
   (slot name (type SYMBOL))
   (slot type (type SYMBOL))
   (slot team-color (type SYMBOL))
   (slot zone (type SYMBOL))
   (slot rotation (type INTEGER))
   (slot state (type SYMBOL))
)

(deftemplate ring-assignment
  (slot machine (type SYMBOL))
  (multislot colors (type SYMBOL))
)

(deftemplate ring-spec
  (slot color (type SYMBOL))
  (slot cost (type INTEGER))
)

(deftemplate robot
  (slot name (type SYMBOL))
  (slot number (type INTEGER))
  (slot state (type SYMBOL) (allowed-values ACTIVE MAINTENANCE))
  (slot is-busy (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
)

(deftemplate order
  (slot id (type INTEGER))
  (slot name (type SYMBOL))
  (slot workpiece (type SYMBOL))
  (slot complexity (type SYMBOL))

  (slot base-color (type SYMBOL))
  (multislot ring-colors (type SYMBOL))
  (slot cap-color (type SYMBOL))

  (slot quantity-requested (type INTEGER))
  (slot quantity-delivered (type INTEGER))
  (slot quantity-delivered-other (type INTEGER))

  (slot delivery-begin (type INTEGER))
  (slot delivery-end (type INTEGER))
  (slot competitive (type SYMBOL))
)

(deftemplate protobuf-peer
  (slot name (type SYMBOL))
  (slot peer-id (type INTEGER))
)

(deftemplate current-rcll-agent-task-id
   (slot robot (type SYMBOL))
   (slot task-id (type INTEGER))
)

(deftemplate action-task-executor-enable
" Define this for plan actions that should be handled by the protobuf
  executor that sends agent task messages to a suitable simulator.
  The messages are created based on the agent task descriptions that are used
  in beacon signals.
"
  (slot name (type SYMBOL) (default ?NONE))
)


(deftemplate rcll-agent-task
  (slot task-id (type INTEGER))
  (slot robot (type SYMBOL))
  (slot robot-id (type INTEGER) (allowed-values  1 2 3) (default 1))
  (slot task-type (type SYMBOL)
    (allowed-values UNSET Move Retrieve Deliver BufferStation ExploreWaypoint)
    (default UNSET))
  (slot machine (type SYMBOL)
   (allowed-values UNSET
    C-BS C-CS1 C-CS2 C-RS1 C-RS2 C-DS C-SS
    M-BS M-CS1 M-CS2 M-RS1 M-RS2 M-DS M-SS
   )
   (default UNSET))
  (slot order (type SYMBOL) (default UNSET))
  (slot side (type SYMBOL)
    (allowed-values UNSET INPUT OUTPUT LEFT MIDDLE RIGHT SLIDE)
    (default UNSET))
  (slot waypoint (type SYMBOL) (default UNSET))
  (slot workpiece (type SYMBOL) (default UNSET))
  (multislot workpiece-colors (type SYMBOL) (default (create$)))
  (slot outcome (type SYMBOL) (allowed-values UNKNOWN FAILED CANCELLED SUCCEEDED))
  (slot goal-id (type SYMBOL) (default UNSET))
  (slot plan-id (type SYMBOL) (default UNSET))
  (slot action-id (type INTEGER))
)


(deftemplate service-request-meta
  (slot service (type STRING))
  (slot request-id (type INTEGER))
  (slot meta (type SYMBOL))
)

(deftemplate pddl-fluent
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL))
  (slot instance (type SYMBOL))
)

(deftemplate pddl-numeric-fluent
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL))
  (slot value (type FLOAT))
  (slot instance (type SYMBOL))
)
