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

(defrule action-task-connect-receiver-of-sim
  "Enable peer connection to the simulator"
  (confval (path "/rcll-simulator/enabled") (value TRUE))
  (confval (path "/rcll-simulator/host") (value ?peer-address))
  (confval (path "/rcll-simulator/robot-recv-ports") (is-list TRUE) (list-value $?recv-ports))
  (confval (path "/rcll-simulator/robot-send-ports") (is-list TRUE) (list-value $?send-ports))
  (not (protobuf-peer (name robot1)))
  (not (executive-finalize))
  =>
  (printout info "Enabling robot simulation peers" crlf)
  (if (<> (length$ ?recv-ports) (length$ ?send-ports)) then
    (printout error "Expected number or recv ports to be equal to send ports for simulator robots (" (length$ ?recv-ports) " != "(length$ ?send-ports) ")" crlf)
   else
    (loop-for-count (?i (length$ ?recv-ports)) do
      (bind ?peer-id (pb-peer-create-local ?peer-address (string-to-field (nth$ ?i ?send-ports)) (string-to-field (nth$ ?i ?recv-ports))))
      (assert (protobuf-peer (name (sym-cat "ROBOT" ?i)) (peer-id ?peer-id)))
	  (assert (current-rcll-agent-task-id (robot (sym-cat "ROBOT" ?i)) (task-id 0)))
    )
  )
)

(defrule action-task-register
  (not (action-task-executor-enable))
  (confval (path "/rcll-simulator/enabled") (value TRUE))
  =>
  (assert (action-task-executor-enable (name move))
          (action-task-executor-enable (name go-wait))
          (action-task-executor-enable (name wp-get-shelf))
          (action-task-executor-enable (name wp-get))
          (action-task-executor-enable (name wp-put-slide-cc))
          (action-task-executor-enable (name wp-put))
  )
)

; (defrule action-task-set-on-waiting
; " Override skiller invocation for actions with an action-task mapping by
;   putting them in state WAITING before the skiller is invoked.
; "
;   (declare (salience ?*SALIENCE-HIGH*))
;   ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (state PENDING)
;                       (action-name ?action-name) (robot ?robot-str) (executable TRUE))
;   (action-task-executor-enable (name ?action-name))
;   (protobuf-peer (name ?robot))
;   =>
;   (modify ?pa (state WAITING))
; )

; (defrule action-task-send-command
; " Create an AgentTask protobuf message and send it to the simulator peer.
; "
;   (action-task-executor-enable (name ?action-name))
;   (current-rcll-agent-task-id (robot ?robot) (task-id ?task-seq))
;   ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
;            (state WAITING|RUNNING) (action-name ?action-name))
;   ?at <- (rcll-agent-task (task-id ?task-seq) (robot ?robot) (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?id))
;   (protobuf-peer (name ?robot) (peer-id ?peer-id))
;   (game-state (state RUNNING) (phase EXPLORATION|PRODUCTION) (team-color ?team-color&~NOT-SET))
;   =>
;   (bind ?task-msg (create-task-msg ?at ?team-color))
;   (if ?task-msg
;    then
;     (pb-send ?peer-id ?task-msg)
;     (pb-destroy ?task-msg)
;     (modify ?pa (state RUNNING))
;    else
;     (modify ?pa (state FAILED) (error-msg (str-cat "Failed to create agent-task message for " ?action-name)))
;   )
; )

; (defrule agent-task-recv-AgentTask-for-running-action
;   ?pf <- (protobuf-msg (type "llsf_msgs.AgentTask") (ptr ?task-msg))
;   (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
;            (state RUNNING) (action-name ?action-name))
;   (action-task-executor-enable (name ?action-name))
;   ?at <- (rcll-agent-task (task-id ?task-seq) (robot ?robot) (robot-id ?robot-id)
;     (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?id)
;     (outcome UNKNOWN)
;   )
;   (not (rcll-agent-task (robot ?robot) (robot-id ?robot-id)
;     (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?id)
;     (outcome FAILED)
;   ))
;   (test (eq ?robot-id (pb-field-value ?task-msg "robot_id")))
;   ?curr-task-id <- (current-rcll-agent-task-id (robot ?robot) (task-id ?task-seq))
;   =>
;   (bind ?task (pb-field-value ?task-msg "task_id"))
;   (if (eq ?task ?task-seq) then
;     (bind ?robot-num (pb-field-value ?task-msg "robot_id"))
;     (bind ?team-col (pb-field-value ?task-msg "team_color"))
;     (bind ?task-outcome UNKNOWN)
;     (if (pb-has-field ?task-msg "cancelled") then
;       (bind ?cancelled (pb-field-value ?task-msg "cancelled"))
;       (if ?cancelled then
;         (printout warn "Agent Task for " ?action-name " got cancelled" crlf))
;         (bind ?task-outcome CANCELLED)
;     )
;     (if (pb-has-field ?task-msg "successful") then
;       (bind ?successful (pb-field-value ?task-msg "successful"))
;       (if ?successful then
;         (bind ?task-outcome EXECUTION-SUCCEEDED)
;        else
;         (bind ?task-outcome EXECUTION-FAILED)
;       )
;     )
;     (if (neq ?task-outcome UNKNOWN) then
;       (modify ?at (outcome ?task-outcome))
;       (modify ?curr-task-id (task-id (+ ?task-seq 1)))
;     )
;    else
;     (if (> ?task ?task-seq) then
;       (bind ?robot-num (pb-field-value ?task-msg "robot_id"))
;       (bind ?team-col (pb-field-value ?task-msg "team_color"))
;       (printout warn "Received feedback for future task!" crlf)
;       (printout warn ?goal-id " " ?plan-id " " ?id " " ?action-name crlf)
;       (printout warn ?task-seq " " ?robot crlf)
;       (printout warn  ?task " " ?robot-num " " ?team-col crlf)
;     )
;     ; old msg is periodically sent, so just ignore it
;   )
;   (retract ?pf)
; )
; 
; (defrule agent-task-action-done
;   ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
;            (state RUNNING) (action-name ?action-name))
;   (action-task-executor-enable (name ?action-name))
;   (not
;     (rcll-agent-task (task-id ?task-seq) (robot ?robot) (robot-id ?robot-id)
;       (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?id) (outcome ~SUCCEEDED)
;     )
;   )
;   =>
;   (modify ?pa (state EXECUTION-SUCCEEDED))
; )
; 
; (defrule agent-task-action-failed
;   ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
;            (state RUNNING) (action-name ?action-name))
;   (action-task-executor-enable (name ?action-name))
;   (rcll-agent-task (task-id ?task-seq) (robot ?robot) (robot-id ?robot-id)
;       (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?id) (outcome CANCELLED| SUCCEEDED)
;   )
;   =>
;   (modify ?pa (state EXECUTION-FAILED))
; )

(defrule agent-task-recv-AgentTask-no-action
  (declare (salience ?*SALIENCE-LOW*))
  ?pf <- (protobuf-msg (type "llsf_msgs.AgentTask") (ptr ?task-msg))
  =>
  (retract ?pf)
)
