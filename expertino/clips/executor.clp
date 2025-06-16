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
  ?aa <- (agenda-action-item (action ?action-id) (execution-state SELECTED) (worker ?worker))
  (not (executor (pddl-action-id ?action-id)))
  ;TODO agenda-action-item is also supposed to give an assigned worker to the action
  =>
  (bind ?executor-id (sym-cat EXECUTOR-(gensym*)))
  (assert (executor (id ?executor-id) (pddl-action-id ?action-id) (worker ?worker) (state INIT)))
  (assert (executor-monitor (id (sym-cat EXECUTOR-MONITOR-(gensym*))) (executor-id ?executor-id)))
)

(defrule executor-accepted
  ?ex <- (executor (id ?ex-id) (state ACCEPTED) (pddl-action-id ?action-id))
  ?mon <- (executor-monitor (executor-id ?ex-id))
  (pddl-action (id ?action-id) (params $?action-params))
  ?aa <- (agenda-action-item (action ?action-id) (execution-state SELECTED))
  =>
  (modify ?aa (execution-state EXECUTING))
  (modify ?mon (feedback-code ?*EXECUTOR-ACTION-ACCEPTED*))
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

(defrule executor-succeed-agent-worker
  ?ex <- (executor (id ?ex-id) (pddl-action-id ?action-id) (worker AGENT) (state INIT))
  ?mon <- (executor-monitor (executor-id ?ex-id))
  ?aa <- (agenda-action-item (action ?action-id) (execution-state SELECTED))
  =>
  (printout t "Executor: Signaling success for AGENT action " ?action-id " to monitor." crlf)
  (modify ?mon (feedback-code ?*AGENT-ACTION-SUCCESS*))
  (modify ?aa (execution-state EXECUTING))
  
)
