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

  (slot state (type SYMBOL) (default OPEN) (allowed-values OPEN ACTIVE COMPLETED CANCELLED))
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

(deftemplate pddl-manager
" Store information on the external pddl manager.
"
  (slot node (type STRING) (default "/pddl_manager"))
)

(deftemplate pddl-goal-fluent
" Facts to represent goal conditions for planning.
  Each fact of this template represent one positive boolean fluent in a
  goal condition.
  Negative goal conditions are currently not supported.
"
  (slot instance (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
)

(deftemplate pddl-goal-numeric-fluent
" Facts to represent goal conditions for planning.
  Each fact of this template represent one numeric fluent in a goal condition
  with exactly the value specified in here.
  Note that this is a rather limited representation for numeric fluent
  conditions and can therefore only represent a subset of valid conditions.
"
  (slot instance (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
  (slot value (type FLOAT))
)

(deftemplate pddl-set-goals
" Interface for set-goals.clp
  Assert a fact of this type in order to actually register the respective goal
  conditions represented by pddl-goal-(numeric-)fluent facts with the external
  pddl manager.
  @slot instance: pddl instance to which the respective goal conditions are
        added.
  Slots set automatically:
  @slot state:
   - PENDING: The goal is not updated with the pddl manager yet.
   - DONE: The goal is updated with the pddl manager.
   - ERROR: The goal is not (or only partially) updated.
            Note that this leaves the set goal in an undefined state,
            hence it is advised to clear the goal before proceeding.
  @slot error: provide information on encountered errors.
"
  (slot instance (type SYMBOL))
  (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
  (slot error (type STRING))
)

(deftemplate pddl-clear-goals
" Interface for clear-goals.clp
  Assert a fact of this type in order to clear all goal conditions of a given
  pddl instance with the external pddl manager.
  @slot instance: pddl instance which should clear it's goal conditions.
  Slots set automatically:
  @slot state:
   - PENDING: The goal is not cleared in the pddl manager yet.
   - DONE: The goal is cleared with the pddl manager.
   - ERROR: The goal is not cleared due to some error.
  @slot error: provide information on encountered errors.
"
  (slot instance (type SYMBOL))
  (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
  (slot error (type STRING))
)

(deftemplate pddl-get-fluents
" Interface for get-fluents.clp
  Assert a fact of this type in order to fetch all positive boolean fluents
  of a given pddl instance with the external pddl manager.
  This results in the automatic assertion of all positive boolean fluents
  (pddl-fluent template facts) currently present in the given pddl instance.
  @slot instance: pddl instance from which the fluents are fetched.
  Slots set automatically:
  @slot state:
   - PENDING: The fluents were not fetched yet.
   - DONE: The fluents were successfully retrieved
   - ERROR: The fluents were not fetched due to an error.
  @slot error: provide information on encountered errors.
"
  (slot instance (type SYMBOL))
  (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
  (slot error (type STRING))
)

(deftemplate pddl-get-numeric-fluents
" Interface for get-fluents.clp
  Assert a fact of this type in order to fetch all numeric fluents
  of a given pddl instance with the external pddl manager.
  This results in the automatic assertion of all numeric fluents
  (pddl-numeric-fluent template facts) currently present in the given pddl
  instance.
  @slot instance: pddl instance from which the fluents are fetched.
  Slots set automatically:
  @slot state:
   - PENDING: The fluents were not fetched yet.
   - DONE: The fluents were successfully retrieved
   - ERROR: The fluents were not fetched due to an error.
  @slot error: provide information on encountered errors.
"
  (slot instance (type SYMBOL))
  (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
  (slot error (type STRING))
)

(deftemplate pddl-instance
" Interface for instances.clp
  Assert a fact of this type to initialize a pddl instance with the external pddl manager.
  @slot name: unique name to refer to when using this instance
  @slot domain: name of a domain.pddl file to be loaded.
  @slot problem: optional name of the problem.pddl, leave empty if no problem should be loaded initially.
  Slots set automatically:
  @slot state:
   - PENDING: The instance was not registered yet.
   - LOADED: The instance is loaded and ready for usage.
   - ERROR: The fluents were not fetched due to an error.
  @busy-with: Indicates the current operation
  @slot error: provide information on encountered errors.
"
  (slot name (type SYMBOL))
  (slot domain (type STRING))
  (slot problem (type STRING))
  (slot directory (type STRING))
  (slot state (type SYMBOL) (allowed-values PENDING LOADED ERROR) (default PENDING))
  (slot busy-with (type SYMBOL) (allowed-values FALSE OBJECTS FLUENTS ACTION-EFFECTS CLEAR-GOALS SET-GOALS CHECK-CONDITIONS GET-FLUENTS GET-NUMERIC-FLUENTS GET-ACTION-NAMES) (default FALSE))
  (slot error (type STRING))
)

(deftemplate pending-pddl-object
" Interface for objects.clp
  Assert a fact of this type in order to indicate that an object needs to be
  added to/removed from a pddl instance.
  @slot instance: pddl instance to add the object to.
  @slot name: name of the object.
  @slot type: type of the object.
  @slot delete: if true, remove the object, else add it.
  Slots set automatically:
  @slot state:
   - PENDING: The object was not added yet.
   - WAITING: The object is about to be added and is waiting for confirmation.
   - ERROR: The object might not have been fetched due to an error.
   - ON-HOLD: Unused state that can be set in order to defer the object update
     to a later time (by switching it manually to PENDING).
  @slot error: provide information on encountered errors.
"
  (slot instance (type SYMBOL))
  (slot name (type SYMBOL))
  (slot type (type SYMBOL))
  (slot delete (type SYMBOL) (allowed-values FALSE TRUE) (default FALSE))
  (slot request-id (type INTEGER))
  (slot state (type SYMBOL) (allowed-values PENDING WAITING ERROR ON-HOLD) (default PENDING))
)

(deftemplate pending-pddl-fluent
" Interface for fluents.clp
  Assert a fact of this type in order to indicate that a fluent needs to be
  added to/removed from a pddl instance.
  Acts as a transient layer to pddl-fluent facts to make sure the CLIPS
  representation stays consistant with the externally managed pddl instance.
  @slot instance: pddl instance to add the fluent to.
  @slot name: name of the fluent.
  @slot params: parameters of the fluent.
  @slot delete: if true, remove the fluent, else add it.
  Slots set automatically:
  @slot request-id: id of the associated ros service request
  @slot state:
   - PENDING: The fluent was not added yet.
   - WAITING: The fluent is about to be added and is waiting for confirmation.
   - ERROR: The fluent might not have been fetched due to an error.
   - ON-HOLD: Unused state that can be set in order to defer the fluent update
     to a later time (by switching it manually to PENDING).
"
  (slot instance (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
  (slot delete (type SYMBOL) (allowed-values FALSE TRUE) (default FALSE))
  (slot request-id (type INTEGER))
  (slot state (type SYMBOL) (allowed-values PENDING WAITING ERROR ON-HOLD) (default PENDING))
)

(deftemplate pending-pddl-numeric-fluent
" Interface for numeric-fluents.clp
  Assert a fact of this type in order to indicate that a numeric fluent needs
  to be added to/removed from a pddl instance.
  Acts as a transient layer to pddl-numeric-fluent facts to make sure the CLIPS
  representation stays consistant with the externally managed pddl instance.
  @slot instance: pddl instance to add the fluent to.
  @slot name: name of the fluent.
  @slot params: parameters of the fluent.
  @slot value: value of the fluent.
  Slots set automatically:
  @slot request-id: id of the associated ros service request
  @slot state:
   - PENDING: The fluent was not added yet.
   - WAITING: The fluent is about to be added and is waiting for confirmation.
   - ERROR: The fluent might not have been fetched due to an error.
   - ON-HOLD: Unused state that can be set in order to defer the fluent update
     to a later time (by switching it manually to PENDING).
"
  (slot instance (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
  (slot value (type FLOAT))
  (slot request-id (type INTEGER))
  (slot state (type SYMBOL) (allowed-values PENDING WAITING ERROR ON-HOLD) (default PENDING))
)

(deftemplate pddl-fluent
" Represents a (boolean) fluent in a pddl instance.
  Do not retract/assert/modify facts of this type directly, rather use
  request-pddl-fluent facts to indicate the desired change.
  This ensures that the external pddl manager stays in sync with the CLIPS
  model.
  @slot instance: pddl instance to add the fluent to.
  @slot name: name of the fluent.
  @slot params: parameters of the fluent.
"
  (slot instance (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
)

(deftemplate pddl-numeric-fluent
" Represents a numeric fluent in a pddl instance.
  Do not retract/assert/modify facts of this type directly, rather use
  request-pddl-numeric-fluent facts to indicate the desired change.
  This ensures that the external pddl manager stays in sync with the CLIPS
  model.
  @slot instance: pddl instance to add the fluent to.
  @slot name: name of the fluent.
  @slot params: parameters of the fluent.
  @slot value: value of the fluent.
"
  (slot instance (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
  (slot value (type FLOAT))
)

(deftemplate pddl-plan
  (slot id (type SYMBOL))
  (slot instance (type SYMBOL))
  (slot duration (type FLOAT))
  (slot state (type SYMBOL) (allowed-values PENDING EXECUTING) (default PENDING))
)

(deftemplate pddl-action
" Represents a grounded pddl action in a pddl instance.
  @slot instance: pddl instance belonging to the action.
  @slot name: name of the action.
  @slot params: parameters of the  action.
  @slot plan-order-class: partial order class of the action in theplan
  @slot: planned-start-time: start time of the action in the plan
  @slot: planned-duration: assumed duration of the action according to plan
"
  (slot instance (type SYMBOL))
  (slot id (type SYMBOL)) ; this should be a globally unique ID
  (slot plan (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
  (slot plan-order-class (type INTEGER))
  (slot planned-start-time (type FLOAT))
  (slot planned-duration (type FLOAT))
)

(deftemplate pddl-action-apply-effect
" Apply the effect of a grounded pddl action.
  @slot instance: pddl instance belonging to the action.
  TODO: should this reference a pddl-action or copy the values like now?
  @slot action: id of the action.
  @slot state: TBD
"
  (slot instance (type SYMBOL))
  (slot action (type SYMBOL))
  (slot effect-type (type SYMBOL) (allowed-values ALL START END) (default ALL))
  (slot state (type SYMBOL) (allowed-values PENDING WAITING START-EFFECT-APPLIED DONE ERROR) (default PENDING))
)

(deftemplate pddl-action-precondition
  (slot instance (type SYMBOL))
  (slot plan (type SYMBOL))
  (slot id (type SYMBOL))
  (slot context (type SYMBOL))
  (slot state (type SYMBOL) (allowed-values PENDING CHECK-PRECONDITION PRECONDITION-SAT PRECONDITION-UNSAT) (default PENDING))
  (multislot unsatisfied-preconditions (type STRING) (default (create$)))
)

(deftemplate agenda
  (slot plan (type SYMBOL))
  (slot class-selection (type INTEGER) (default 0))
  (slot class-relaxation (type INTEGER) (default 1))
  (slot state (type SYMBOL) (allowed-values ACTIVE INACTIVE) (default INACTIVE))
)

(deftemplate agenda-action-item
  (slot plan (type SYMBOL))
  (slot action (type SYMBOL))
  (slot execution-state (type SYMBOL) (allowed-values INITIAL UNSAT PENDING SELECTED EXECUTING COMPLETED ERROR EFFECTS-APPLIED) (default INITIAL))
  (multislot priority (type INTEGER) (default (create$ 0)))
  (slot worker-type (type SYMBOL) (allowed-values ROBOT REFBOX))
  (slot worker (type SYMBOL) (default UNSET))
)

(deftemplate pddl-action-names
" Retrieve the list of action names.
  @slot instance: pddl instance to retrieve the action names for.
  Slots set automatically:
  @multislot action-names: retrieved list of action names
  @slot state:
   - PENDING: The action names were not retrieved yet.
   - ERROR: The names were not fetched due to an error.
   - DONE: The action-names slot now is filled properly.
  @slot error: provide information on encountered errors.
"
  (slot instance (type SYMBOL))
  (multislot action-names (type SYMBOL) (default (create$)))
  (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
  (slot error (type STRING))
)

(deftemplate planning-filter
" This currently mainly is a transient layer betweeen the general pddl interface and our domain-specific usage.
  Can be extended later in case different kind of planning filters should be used or if planning is used in varying contexts.
"
  (multislot action-names (type SYMBOL) (default (create$ )))
)

(deftemplate executor
" Interface to the execution layer. Asserted when an (action) is ready to be executed by a worker. 
  Once the action has started executing, at-start effects are applied. Throughout the duration of action,
  feedback is received and corresponding effects are applied.
  @slot id: id of the executor.
  @slot worker: id of the worker which could be a robot or a refbox machine. For now, assuming ROBOT1, ROBOT2, ROBOT3 or REFBOX.
  @slot pddl-action-id: the id of the pddl-action to be executed.
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
  (slot pddl-action-id (type SYMBOL))
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
