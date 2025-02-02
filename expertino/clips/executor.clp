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

(defrule executor-create
  ?pa <- (pddl-action (id ?action-id))
  ?aa <- (agenda-action-item (action ?action-id) (execution-state SELECTED))
  (not (executor (pddl-action-id ?action-id)))
  ;TODO agenda-action-item is also supposed to give an assigned worker to the action
  =>
  (assert (executor (id (sym-cat EXECUTOR-(gensym*))) (pddl-action-id ?action-id) (state INIT)))
)

;(defrule executor-select-robot
;  ?ex <- (executor (id ?ex-id) (state INIT) (pddl-action-id ?action-id))
;  (confval (path "/pddl/actions/robot") (is-list TRUE) (list-value $?robot-actions))
;  ?pa <- (pddl-action (id ?action-id) (params $?action-params)
;           (name ?action-name&:(member$ (str-cat ?action-name) ?robot-actions)))
;  ?aa <- (agenda-action-item (action ?action-id) (execution-state SELECTED))
;  ; (robot (id ?robot-id))
;  =>
;  ;TODO select the robot according to a strategy
;  (modify ?ex (worker ROBOT1) (state ASSIGNED))
;)
;
;(defrule executor-select-refbox
;  ?ex <- (executor (id ?ex-id) (pddl-action-id ?action-id) (state INIT))
;  (confval (path "/pddl/actions/refbox") (is-list TRUE) (list-value $?refbox-actions))
;  ?pa <- (pddl-action (id ?action-id) (params $?action-params)
;           (name ?action-name&:(member$ (str-cat ?action-name) ?refbox-actions)))
;  ?aa <- (agenda-action-item (action ?action-id) (execution-state SELECTED))
;  (protobuf-peer (name refbox-private))
;  =>
;  (modify ?ex (worker REFBOX) (state ASSIGNED))
;) 

(defrule executor-accepted
  (executor (id ?ex-id) (state ACCEPTED) (pddl-action-id ?action-id))
  (pddl-action (id ?action-id) (params $?action-params))
  ?aa <- (agenda-action-item (action ?action-id) (execution-state SELECTED))
  =>
  (modify ?aa (execution-state EXECUTING))
)

(defrule executor-succeeded
  (executor (id ?ex-id) (state SUCCEEDED) (pddl-action-id ?action-id))
  (pddl-action (id ?action-id) (params $?action-params))
  ?aa <- (agenda-action-item (action ?action-id) (execution-state EXECUTING))
  =>
  (modify ?aa (execution-state COMPLETED))
)

(defrule executor-failed
  (executor (id ?ex-id) (state ABORTED) (pddl-action-id ?action-id))
  (pddl-action (id ?action-id) (params $?action-params))
  ?aa <- (agenda-action-item (action ?action-id) (execution-state EXECUTING))
  =>
  ;TODO set appropriate error message
  (modify ?aa (execution-state ERROR))
)
