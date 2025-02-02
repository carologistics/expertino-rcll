; the action selection picks one action among the actions that are currently on the 
; agenda and selects the next action to execute. The selection is based on
; the priority values of the agenda items. 
; Only one agenda item is selected at a time.


(defrule agenda-action-sat-check-start
  (agenda-action-item (plan ?plan-id) (action ?action-id) (execution-state INITIAL))
  (not (pddl-action-precondition (plan ?plan-id) (id ?action-id) (context AGENDA-SELECTION)))
  =>
  (assert (pddl-action-precondition (plan ?plan-id) (id ?action-id) (state PENDING) (context AGENDA-SELECTION)))
)

(defrule agenda-action-sat-check-reset
  ?action <- (agenda-action-item (plan ?plan-id) (action ?action-id) (execution-state PENDING|UNSAT))
  ?precon <- (pddl-action-precondition (plan ?plan-id) (id ?action-id) (state PRECONDITION-UNSAT|PRECONDITION-SAT|PENDING) (context AGENDA-SELECTION))
  (agenda-action-item (plan ?plan-id) (action ?completed-action-id&:(neq ?completed-action-id ?action-id)) (execution-state EFFECTS-APPLIED))
  =>
  (retract ?precon)
  (modify ?action (execution-state INITIAL))
)

(defrule agenda-action-sat-check-apply-results
  ?action <- (agenda-action-item (action ?action-id) (execution-state INITIAL))
  (pddl-action-precondition (plan ?plan-id) (id ?action-id) (state ?state&PRECONDITION-UNSAT|PRECONDITION-SAT) (context AGENDA-SELECTION))
  =>
  (if (eq ?state PRECONDITION-SAT)
    then
      (modify ?action (execution-state PENDING))
      (printout green "Action " ?action-id " set to PENDING" crlf)
    else
      (modify ?action (execution-state UNSAT))
      (printout red "Action " ?action-id " set to UNSAT" crlf)
  )
)

(defrule action-agenda-item-selection
    (not (agenda-action-item (action ?some-action-id) (plan ?plan-id) (execution-state SELECTED)))
    ?action <- (agenda-action-item (action ?action-id) (plan ?plan-id) (priority $?priority) (execution-state PENDING))
    (not
        (and
            (agenda-action-item (action ?other-action-id) (plan ?plan-id) (priority $?other-priority) (execution-state PENDING))
            (test (> (+ (expand$ ?priority)) (+ (expand$ ?other-priority))))
        )
    )
    (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
    (not (agenda-action-item (execution-state INITIAL)))
    =>
    (modify ?action (execution-state SELECTED))
    (printout green "Selected action " ?action-name "[" ?action-id "]" ?action-params " from agenda" crlf)
)

(defrule action-agenda-item-update-selection
    ?action <- (agenda-action-item (action ?action-id) (plan ?plan-id) (priority $?priority) (execution-state SELECTED))
    ?other-action <- (agenda-action-item (action ?other-action-id) (plan ?plan-id) (priority $?other-priority) (execution-state PENDING))
    (test (> (+ (expand$ ?priority)) (+ (expand$ ?other-priority))))
    (pddl-action (id ?other-action-id) (name ?action-name) (params $?action-params))
    =>
    (modify ?action (execution-state PENDING))
    (printout yellow "Deselected action " ?action-id " from agenda" crlf)
    (modify ?other-action (execution-state SELECTED))
    (printout green "Selected action " ?action-name "[" ?other-action-id "]" ?action-params " from agenda" crlf)
)



