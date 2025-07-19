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

(defrule agent-task-connect-receiver-of-sim
  "Enable peer connection to the simulator"
  (confval (path "/rcll-simulator/enabled") (value TRUE))
  (confval (path "/rcll-simulator/host") (value ?peer-address))
  (confval (path "/rcll-simulator/robot-recv-ports") (is-list TRUE) (list-value $?recv-ports))
  (confval (path "/rcll-simulator/robot-send-ports") (is-list TRUE) (list-value $?send-ports))
  (confval (path "/rcll-simulator/robot-ids") (is-list TRUE) (list-value $?robot-ids))
  (not (and
         (protobuf-peer (name ROBOT1))
         (protobuf-peer (name ROBOT2))
         (protobuf-peer (name ROBOT3))
  ))
  (not (executive-finalize))
  =>
  (printout info "Enabling robot simulation peers" crlf)
  (if (<> (length$ ?recv-ports) (length$ ?send-ports) (length$ ?robot-ids)) then
    (printout error "Expected number or recv ports to be equal to send ports for simulator robots (" (length$ ?recv-ports) " != "(length$ ?send-ports) ")" crlf)
   else
    (loop-for-count (?i (length$ ?recv-ports)) do
      (bind ?robot (sym-cat "ROBOT" (nth$ ?i ?robot-ids)))
      (bind ?peer-id (pb-peer-create-local ?peer-address (string-to-field (nth$ ?i ?send-ports)) (string-to-field (nth$ ?i ?recv-ports))))
      (assert (protobuf-peer (name ?robot) (peer-id ?peer-id)))
      (assert (current-rcll-agent-task-id (robot ?robot) (task-id 1)))
      (assert (worker (id ?robot) (state IDLE) (type ROBOT)))
    )
  )
)

(defrule agent-task-send-command
 " Create an AgentTask protobuf message and send it to the simulator peer.
 "
   (current-rcll-agent-task-id (robot ?robot) (task-id ?task-seq))
   ?at <- (rcll-agent-task (task-id ?task-seq) (robot ?robot) (executor-id ?ex-id) (outcome UNKNOWN) (sent ?time) (ack ?ack-time))
   ?ex <- (executor (id ?ex-id) (worker ?robot) (state ?state))
   (protobuf-peer (name ?robot) (peer-id ?peer-id))
   (game-state (state RUNNING) (phase EXPLORATION|PRODUCTION) (team-color ?team-color&~NOT-SET))
   (game-time ?gt)
   (test (>= (- ?gt ?time) 2))
   (test (>= ?ack-time ?time))
   =>
   (bind ?task-msg (create-task-msg ?at ?team-color))
   (if ?task-msg
    then
     (pb-send ?peer-id ?task-msg)
     (pb-destroy ?task-msg)
     (modify ?at (sent ?gt))
     (if (eq ?state REQUESTED)
      then (modify ?ex (state ACCEPTED))
     )
    else
     (modify ?ex (state ABORTED))
     (modify ?at (outcome FAILED))
     ;TODO set error message and feedback
   )
 )

(defrule agent-task-recv-AgentTask-for-running-action
  ?pf <- (protobuf-msg (type "llsf_msgs.AgentTask") (ptr ?task-msg))
  ?at <- (rcll-agent-task (task-id ?task-seq) (robot ?robot)
    (outcome UNKNOWN) (task-name ?task-name) (executor-id ?ex-id) (sent ?sent-time)
    (ack ?ack-time)
  )
  ?ex <- (executor (id ?ex-id))
  (not (rcll-agent-task (robot ?robot)
    (outcome FAILED)
  ))
  (test (eq (string-to-field (sub-string (str-length ?robot) (str-length ?robot) ?robot))
    (pb-field-value ?task-msg "robot_id")))  
  ?curr-task-id <- (current-rcll-agent-task-id (robot ?robot) (task-id ?task-seq))
  (game-time ?gt)
  =>
  (bind ?task (pb-field-value ?task-msg "task_id"))
  (if (eq ?task ?task-seq) then
    (bind ?robot-num (pb-field-value ?task-msg "robot_id"))
    (bind ?team-col (pb-field-value ?task-msg "team_color"))
    (bind ?task-outcome UNKNOWN)
    (if (pb-has-field ?task-msg "cancelled") then
      (bind ?cancelled (pb-field-value ?task-msg "cancelled"))
      (if ?cancelled then
        (printout warn "Agent Task for " ?task-name " got cancelled" crlf)
        (bind ?task-outcome CANCELLED))
    )
    (if (pb-has-field ?task-msg "successful") then
      (bind ?successful (pb-field-value ?task-msg "successful"))
      (if ?successful then
        (bind ?task-outcome SUCCEEDED)
       else
        (bind ?error-code (pb-field-value ?task-msg "error_code"))
        (if (neq ?error-code 0) then
          (bind ?task-outcome FAILED)
          (printout warn "agent-task " ?task-name " with id " ?task " for " ?robot " got aborted with error code " ?error-code crlf)
        )
        ;(modify ?ex (state ABORTED))
      )
    )
    (if (neq ?task-outcome UNKNOWN) then
      (modify ?at (outcome ?task-outcome))
    )
   else
    (if (> ?task ?task-seq) then
      (bind ?robot-num (pb-field-value ?task-msg "robot_id"))
      (bind ?team-col (pb-field-value ?task-msg "team_color"))
      (printout warn "Received feedback for future task!" crlf)
      (printout warn ?task-seq " " ?robot crlf)
      (printout warn  ?task " " ?robot-num " " ?team-col crlf)
    )
    (if (> (- ?gt ?sent-time) 10) then
       (printout warn "haven't received a feedback for task " ?task "for " ?robot " in 10 seconds") 
    )
    ; old msg is periodically sent, so just ignore it
  )
  (modify ?at (ack ?gt))
  (retract ?pf)
)

(defrule agent-task-recv-AgentTask-no-action
  (declare (salience ?*SALIENCE-LOW*))
  ?pf <- (protobuf-msg (type "llsf_msgs.AgentTask") (ptr ?task-msg))
  =>
  (retract ?pf)
)
