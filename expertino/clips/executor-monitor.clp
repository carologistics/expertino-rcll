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

; Rule for when an action fails (currently specific to CARRIER-TO-INPUT-FAILED feedback)
(defrule executor-monitor-action-failed
  ?m <- (executor-monitor (executor-id ?ex-id) (feedback-code CARRIER-TO-INPUT-FAILED) (sub-action-id ?sa-id))
  ?ex <- (executor (id ?ex-id) (pddl-action-id ?action-id) (state ~ABORTED)) ; Ensure not already processed
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