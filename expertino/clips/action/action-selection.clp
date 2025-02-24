; the action selection picks one action among the actions that are currently on the 
; agenda and selects the next action to execute. The selection is based on
; the priority values of the agenda items. 
; Only one agenda item is selected at a time.


(defrule agenda-action-sat-check-start
  (agenda-action-item (plan ?plan-id) (action ?action-id) (execution-state INITIAL) (worker-type ?worker-type))
  (not (pddl-action-precondition (plan ?plan-id) (id ?action-id) (context AGENDA-SELECTION)))
  (worker (id ?worker) (state IDLE) (type ?worker-type))
  (pddl-plan (id ?plan-id) (instance ?instance))
  (pddl-instance-update (instance ?instance) (last-updated ?last-update-time))
  =>
  (assert (pddl-action-precondition (plan ?plan-id) (instance ?instance) (id ?action-id) (state PENDING) (context AGENDA-SELECTION) (instance-update ?last-update-time)))
)

(defrule agenda-action-sat-check-reset
  ?action <- (agenda-action-item (plan ?plan-id) (action ?action-id) (execution-state PENDING|UNSAT) (worker-type ?worker-type))
  ?precon <- (pddl-action-precondition (plan ?plan-id) (id ?action-id) (state PRECONDITION-UNSAT|PRECONDITION-SAT|PENDING) (context AGENDA-SELECTION) (instance-update ?update-time))
  (worker (id ?worker) (state IDLE) (type ?worker-type))
  (pddl-plan (id ?plan-id) (instance ?instance))
  (pddl-instance-update (instance ?instance) (last-updated ?last-update-time&:(> ?last-update-time ?update-time)))
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
  ?action <- (agenda-action-item (action ?action-id) (plan ?plan-id) (priority $?priority) (execution-state PENDING)
                 (worker-type ?worker-type&~AGENT))
  (not (agenda-action-item (action ?some-action-id) (plan ?plan-id) (execution-state SELECTED) (worker-type ?worker-type)))
  (not
      (and
          (agenda-action-item (action ?other-action-id) (plan ?plan-id) (priority $?other-priority) (execution-state PENDING) (worker-type ?worker-type))
          (test (> (+ (expand$ ?priority)) (+ (expand$ ?other-priority))))
      )
  )
  (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  (not (agenda-action-item (execution-state INITIAL))) ;all agenda items are precondition checked
  ;assign the worker of the right type with the longest waiting time
  (worker (id ?worker) (state IDLE) (type ?worker-type))
  (worker-idle-timer (worker ?worker) (start-time ?start-time))
  (not 
      (and
          (worker-idle-timer (worker ?other-worker) (start-time ?other-start-time&:(< ?other-start-time ?start-time)))
          (worker (id ?other-worker) (type ?worker-type))
      )
  )
  =>
  (modify ?action (execution-state SELECTED) (worker ?worker))
  (printout green "Selected action " ?action-name "[" ?action-id "]" ?action-params " from agenda" crlf)
  (printout green "Assigned worker  " ?worker " of type " ?worker-type ", waiting since " ?start-time crlf)
)

(defrule action-agenda-item-selection-worker-agent
  ?action <- (agenda-action-item (action ?action-id) (plan ?plan-id) (priority $?priority) (execution-state PENDING)
                 (worker-type ?worker-type&AGENT))
  (not (agenda-action-item (action ?some-action-id) (plan ?plan-id) (execution-state SELECTED) (worker-type ?worker-type)))
  (not
      (and
          (agenda-action-item (action ?other-action-id) (plan ?plan-id) (priority $?other-priority) (execution-state PENDING) (worker-type ?worker-type))
          (test (> (+ (expand$ ?priority)) (+ (expand$ ?other-priority))))
      )
  )
  (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  (not (agenda-action-item (execution-state INITIAL))) ;all agenda items are precondition checked
  =>
  (bind ?worker AGENT)
  (modify ?action (execution-state SELECTED) (worker ?worker))
  (printout green "Selected action " ?action-name "[" ?action-id "]" ?action-params " from agenda" crlf)
  (printout green "Assigned worker  " ?worker " of type " ?worker-type  crlf)
)

(defrule action-agenda-item-update-selection
  ?action <- (agenda-action-item (action ?action-id) (plan ?plan-id) (priority $?priority) (execution-state SELECTED) (worker-type ?worker-type))
  ?other-action <- (agenda-action-item (action ?other-action-id) (plan ?plan-id) (priority $?other-priority) (execution-state PENDING) (worker-type ?worker-type))
  (test (> (+ (expand$ ?priority)) (+ (expand$ ?other-priority))))
  (pddl-action (id ?other-action-id) (name ?action-name) (params $?action-params))
  (worker (id ?worker) (state IDLE) (type ?worker-type))
  (worker-idle-timer (worker ?worker) (start-time ?start-time))
  (not (worker-idle-timer (worker ?other-worker) (start-time ?other-start-time&:(< ?other-start-time ?start-time))))
  =>
  (modify ?action (execution-state PENDING) (worker ?worker))
  (printout yellow "Deselected action " ?action-id " from agenda" crlf)
  (modify ?other-action (execution-state SELECTED) (worker UNSET))
  (printout green "Selected action " ?action-name "[" ?other-action-id "]" ?action-params " from agenda" crlf)
  (printout green "Assigned worker  " ?worker " of type " ?worker-type ", waiting since " ?start-time crlf)
)
