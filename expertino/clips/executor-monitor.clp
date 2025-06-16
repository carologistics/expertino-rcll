; Rule for when carrier-to-input PDDL action succeeds
(defrule executor-monitor-carrier-to-input-succeeded
  ?m <- (executor-monitor (executor-id ?ex-id) (feedback-code CARRIER-TO-INPUT-SUCCESS) (sub-action-id ?sa-id))
  ?ex <- (executor (id ?ex-id) (pddl-action-id ?action-id) (state ~SUCCEEDED)) ; Ensure not already processed
  (pddl-action (id ?action-id) (name carrier-to-input))
  =>
  (printout t "Executor Monitor: Action " ?action-id " (carrier-to-input) SUCCEEDED. Applying END effects." crlf)
  (modify ?ex (state SUCCEEDED))
  (assert (pddl-action-apply-effect (action ?action-id) (effect-type END)))
  (modify ?m (feedback-code NONE)) ; Reset feedback
)

; Rule for when a generic PDDL action succeeds
(defrule executor-monitor-generic-action-succeeded
  ?m <- (executor-monitor (executor-id ?ex-id) (feedback-code ACTION-GENERIC-SUCCESS) (sub-action-id ?sa-id))
  ?ex <- (executor (id ?ex-id) (pddl-action-id ?action-id) (state ~SUCCEEDED)) ; Ensure not already processed
  (pddl-action (id ?action-id) (name ?action-name&~carrier-to-input))
  =>
  (printout t "Executor Monitor: Action " ?action-id " (" ?action-name ") SUCCEEDED. Applying ALL effects." crlf)
  (modify ?ex (state SUCCEEDED))
  (assert (pddl-action-apply-effect (action ?action-id) (effect-type ALL)))
  (modify ?m (feedback-code NONE)) ; Reset feedback
)

(defrule executor-monitor-generic-action-failed
  ?m <- (executor-monitor (executor-id ?ex-id) (feedback-code ACTION-GENERIC-FAILED) (sub-action-id ?sa-id))
  ?ex <- (executor (id ?ex-id) (pddl-action-id ?action-id) (state ABORTED))
  (pddl-action (id ?action-id) (name ?action-name&~carrier-to-input))
  =>
  (printout t "Executor Monitor: Action " ?action-id " (" ?action-name ") FAILED." crlf)
  (modify ?m (feedback-code NONE)) ; Reset feedback
)

(defrule executor-monitor-action-failed
  ?m <- (executor-monitor (executor-id ?ex-id) (feedback-code CARRIER-TO-INPUT-FAILED) (sub-action-id ?sa-id))
  ?ex <- (executor (id ?ex-id) (pddl-action-id ?action-id) (state ABORTED)) ; Ensure not already processed
  (pddl-action (id ?action-id) (name ?action-name))
  =>
  (printout t "Executor Monitor: Action " ?action-id " (" ?action-name ") FAILED." crlf)
  (modify ?ex (state ABORTED))
  (modify ?m (feedback-code NONE)) ; Reset feedback
)

; --- Rules for specific sub-action successes ---

(defrule executor-monitor-subtask-drive-to-src-succeeded
  ?m <- (executor-monitor (executor-id ?ex-id)
                          (feedback-code SUBTASK-DRIVE-TO-SRC-SUCCESS)
                          (sub-action-id ?sa-id&~NONE)) ; Ensure there's a sub-action ID
  ; The ?ex pattern is useful for logging the parent action context.
  ?ex <- (executor (id ?ex-id) (pddl-action-id ?parent-action-id)) 
  =>
  (printout t "Executor Monitor: Sub-action Drive-to-Source for parent " ?parent-action-id " (executor " ?ex-id ") SUCCEEDED." crlf)
  (printout t "Executor Monitor: Applying ALL effects for sub-action " ?sa-id crlf)
  (assert (pddl-action-apply-effect (action ?sa-id) (effect-type ALL)))
  (modify ?m (feedback-code NONE) (sub-action-id NONE)) ; Reset feedback and sub-action-id
)

(defrule executor-monitor-subtask-pick-up-succeeded
  ?m <- (executor-monitor (executor-id ?ex-id)
                          (feedback-code SUBTASK-PICK-UP-SUCCESS)
                          (sub-action-id ?sa-id&~NONE))
  ?ex <- (executor (id ?ex-id) (pddl-action-id ?parent-action-id))
  =>
  (printout t "Executor Monitor: Sub-action Pick-Up for parent " ?parent-action-id " (executor " ?ex-id ") SUCCEEDED." crlf)
  (printout t "Executor Monitor: Applying ALL effects for sub-action " ?sa-id crlf)
  (assert (pddl-action-apply-effect (action ?sa-id) (effect-type ALL)))
  (modify ?m (feedback-code NONE) (sub-action-id NONE))
)

(defrule executor-monitor-subtask-drive-to-dest-succeeded
  ?m <- (executor-monitor (executor-id ?ex-id)
                          (feedback-code SUBTASK-DRIVE-TO-DEST-SUCCESS)
                          (sub-action-id ?sa-id&~NONE))
  ?ex <- (executor (id ?ex-id) (pddl-action-id ?parent-action-id))
  =>
  (printout t "Executor Monitor: Sub-action Drive-to-Destination for parent " ?parent-action-id " (executor " ?ex-id ") SUCCEEDED." crlf)
  (printout t "Executor Monitor: Applying ALL effects for sub-action " ?sa-id crlf)
  (assert (pddl-action-apply-effect (action ?sa-id) (effect-type ALL)))
  (modify ?m (feedback-code NONE) (sub-action-id NONE))
)

(defrule executor-monitor-subtask-place-down-succeeded
  ?m <- (executor-monitor (executor-id ?ex-id)
                          (feedback-code SUBTASK-PLACE-DOWN-SUCCESS)
                          (sub-action-id ?sa-id&~NONE))
  ?ex <- (executor (id ?ex-id) (pddl-action-id ?parent-action-id))
  =>
  (printout t "Executor Monitor: Sub-action Place-Down for parent " ?parent-action-id " (executor " ?ex-id ") SUCCEEDED." crlf)
  (printout t "Executor Monitor: Applying ALL effects for sub-action " ?sa-id crlf)
  (assert (pddl-action-apply-effect (action ?sa-id) (effect-type ALL)))
  (modify ?m (feedback-code NONE) (sub-action-id NONE))
)
;-------------------------------------------------------------------------------------------------------------------------------------

(defrule executor-monitor-refbox-action-direct-succeeded
  ?m <- (executor-monitor (executor-id ?ex-id) 
                          (feedback-code REFBOX-ACTION-DIRECT-SUCCESS)
        )
  ?ex <- (executor (id ?ex-id) 
                    (pddl-action-id ?action-id) 
                    (worker REFBOX) 
                    (state SUCCEEDED)
          )
  (pddl-action (id ?action-id) (name ?action-name))
  =>
  (printout t "Executor Monitor: REFBOX Action " ?action-id " (" ?action-name ") confirmed SUCCEEDED by direct signal. Resetting monitor." crlf)
  (assert (pddl-action-apply-effect (action ?action-id) (effect-type END)))
  (modify ?ex (state SUCCEEDED))
  (modify ?m (feedback-code NONE)) 
)


(defrule executor-monitor-agent-action-immediate-succeeded
  ?m <- (executor-monitor (executor-id ?ex-id) 
                          (feedback-code AGENT-ACTION-SUCCESS)
        )
  ?ex <- (executor (id ?ex-id) 
                    (pddl-action-id ?action-id) 
                    (worker AGENT) 
                    (state INIT)
          )
  (pddl-action (id ?action-id) (name ?action-name))
  =>
  (printout t "Executor Monitor: AGENT Action " ?action-id " (" ?action-name ") processing immediate success signal. Setting SUCCEEDED and applying ALL effects." crlf)
  (modify ?ex (state SUCCEEDED))
  (assert (pddl-action-apply-effect (action ?action-id) (effect-type ALL)))
  (modify ?m (feedback-code NONE)) 
)

(defrule executor-accepted-agenda-action-executing
  ?ex <- (executor (id ?ex-id) (state ACCEPTED) (pddl-action-id ?action-id))
  ?mon <- (executor-monitor (executor-id ?ex-id) (feedback-code EXECUTOR-ACTION-ACCEPTED))
  (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  ?aa <- (agenda-action-item (action ?action-id) (execution-state EXECUTING))
  =>
  (printout t "Executor Monitor: EXECUTOR Agenda Action " ?action-id " (" ?action-name ") accepted and executing. Applying START effects." crlf)
  (assert (pddl-action-apply-effect (action ?action-id) (effect-type START)))
)


(defrule executor-failed
  ?ex <- (executor (id ?ex-id) (pddl-action-id ?action-id) (state ABORTED))
  =>
  (printout t "Executor Monitor: Executor " ?ex-id " FAILED. " crlf)
)