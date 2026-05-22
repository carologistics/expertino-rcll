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

(deftemplate order
  (slot id (type SYMBOL))
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

(deftemplate service-request-meta
  (slot service (type STRING))
  (slot request-id (type INTEGER))
  (slot meta (type SYMBOL))
)

(deftemplate action-feedback
  (slot action-id (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
)

(deftemplate pddl-instance-update
  (slot instance (type SYMBOL))
  (slot last-updated (type FLOAT))
)

(deftemplate executor
" Interface to the execution layer. Asserted when an (action) is ready to be executed by a worker. 
  Once the action has started executing, at-start effects are applied. Throughout the duration of action,
  feedback is received and corresponding effects are applied.
  @slot id: id of the executor.
  @slot worker: id of the worker which could be a robot or a refbox machine. For now, assuming ROBOT1, ROBOT2, ROBOT3 or REFBOX.
  @slot action-id: the id of the pddl-action to be executed.
  @slot state: modelled on the states of ros2 action server responses
   - INIT: initial state before the execution layer is invoked.
   - REQUESTED: the execution layer has been requested to execute the action.
   - ACCEPTED: the execution layer has accepted the request and has started execution.
   - ABORTED: the execution is aborted due to failure.
   - CANCELLED: the execution of action is delibrately cancelled by the user.
   - SUCCEEDED: the execution layer has succeeded in the execution of the action. 
"
  (slot id (type SYMBOL))
  (slot worker (type SYMBOL))
  (slot action-id (type SYMBOL))
  (slot state (type SYMBOL) (allowed-values INIT REQUESTED ACCEPTED ABORTED CANCELLED SUCCEEDED))
)

(deftemplate rcll-agent-task
  (slot task-id (type INTEGER))
  (slot task-name (type SYMBOL))
  (slot robot (type SYMBOL))
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
  (slot executor-id (type SYMBOL))
  (slot retry-count (type INTEGER) (default 0))
  (slot sent (type FLOAT))
  (slot ack (type SYMBOL) (allowed-values FALSE TRUE) (default FALSE))
)

(deftemplate agent-task-list
  (slot id (type SYMBOL))
  (slot pddl-action-id (type SYMBOL))
  (slot executor-id (type SYMBOL))
  (multislot tasks (type SYMBOL) (default (create$)))
  (slot current-task-id (type INTEGER))
  (multislot params (type SYMBOL))
)

(deftemplate worker
  (slot id (type SYMBOL))
  (slot name (type SYMBOL))
  (slot type (type SYMBOL) (allowed-values ROBOT REFBOX))
  (slot state (type SYMBOL) (allowed-values IDLE BUSY RECOVERY))
  (slot refbox-state (type SYMBOL) (allowed-values ACTIVE MAINTENANCE))
)

(deftemplate worker-idle-timer
  (slot worker (type SYMBOL))
  (slot start-time (type FLOAT) (default 0.0))
)

(deftemplate workpiece-for-order
  (slot wp (type SYMBOL))
  (slot order (type SYMBOL))
)

(deftemplate production-strategy-order-filter
  (slot name (type SYMBOL)) 
  (multislot orders (type SYMBOL) (default (create$)))
)

