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
  ?aa <- (agenda-action-item (plan ?plan-id) (action ?action-id) (execution-state SELECTED) (worker ?worker))
  (agenda (plan ?plan-id) (state ACTIVE))
  (not (executor (pddl-action-id ?action-id)))
  ;TODO agenda-action-item is also supposed to give an assigned worker to the action
  =>
  (assert (executor (id (sym-cat EXECUTOR-(gensym*))) (pddl-action-id ?action-id) (worker ?worker) (state INIT)))
)

(defrule executor-accepted
  (executor (id ?ex-id) (state ACCEPTED) (pddl-action-id ?action-id))
  (pddl-action (id ?action-id) (params $?action-params))
  ?aa <- (agenda-action-item (action ?action-id) (execution-state SELECTED))
  =>
  (modify ?aa (execution-state EXECUTING))
  (assert (pddl-action-apply-effect (action ?action-id) (effect-type START)))
  ;(assert (selected-order (order 2)))
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

(defrule executor-agent-worker-replan
  ?ex <- (executor (pddl-action-id ?action-id) (worker AGENT) (state INIT))
  ?aa <- (agenda-action-item (action ?action-id) (execution-state SELECTED))
  =>
  (modify ?ex (state SUCCEEDED))
  (modify ?aa (execution-state COMPLETED))
  (assert (pddl-action-apply-effect (action ?action-id) (effect-type ALL)))
)
