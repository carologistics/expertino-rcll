; The agenda module manages the set of actions currently ready for execution or in progress.
; An agenda is created from a plan that was selected for execution and is updated based on the
; current progress within the partial order of the plan. The agenda is used to determine the
; next actions to execute and to check the preconditions of actions that are ready for execution
; before assigning them to executors. 

(defrule agenda-select-first-plan
  (not (pddl-plan (state EXECUTING)))
  (not (agenda (plan ?)))
  ?plan <- (pddl-plan (id ?plan-id))
  =>
  (assert (agenda (plan ?plan-id) (state ACTIVE)))
)

(defrule agenda-step-ahead
  ?ag <- (agenda (plan ?plan-id) (class-selection ?ordering-class))
  (not
    (and
      (pddl-action (id ?action-id) (plan ?plan-id) (plan-order-class ?ordering-class))
      (not (agenda-action-item (plan ?plan-id) (action ?action-id)))
    )
  )
  (not
    (and
      (pddl-action (id ?action-id) (plan ?plan-id) (plan-order-class ?ordering-class))
      (agenda-action-item (plan ?plan-id) (action ?action-id) (execution-state ~COMPLETED))
    )
  )
  (pddl-action (id ?action-id) (plan ?plan-id) (plan-order-class ?action-order-class&:(> ?action-order-class ?ordering-class)))
  =>
  (modify ?ag (class-selection (+ ?ordering-class 1)))
)

(defrule agenda-add-action
  (agenda (plan ?plan-id) (class-selection ?ordering-class) (class-relaxation ?relaxation))
  (pddl-action (id ?action-id) (plan ?plan-id) (plan-order-class ?ordering-class))
  (not (agenda-action-item (plan ?plan-id) (action ?action-id)))
  =>
  (assert (agenda-action-item (action ?action-id) (plan ?plan-id) (priority 0 1)))
  (printout red "Added action " ?action-id "(ordering " ?ordering-class ") to agenda" crlf)
)

(defrule agenda-add-action-relaxed
  (agenda (plan ?plan-id) (class-selection ?ordering-class-selected) (class-relaxation ?relaxation))
  (pddl-action (id ?action-id) (plan ?plan-id) (plan-order-class ?plan-order-class&:(>= ?ordering-class-selected  (- ?plan-order-class ?relaxation))))
  ?precon <- (pddl-action-precondition (plan ?plan-id) (id ?action-id) (state PRECONDITION-SAT) (context AGENDA-LOOKAHEAD))
  (not (agenda-action-item (plan ?plan-id) (action ?action-id)))
  =>
  (assert (agenda-action-item (action ?action-id) (plan ?plan-id) (priority 0 0)))
  (retract ?precon)
  (printout red "Added action " ?action-id "(" ?plan-order-class ") to agenda (relaxed condition, precondition satisfied)" crlf)
)

(defrule agenda-add-action-relaxed-check
    (agenda (plan ?plan-id) (class-selection ?ordering-class-selected) (class-relaxation ?relaxation))
    (pddl-action (id ?action-id) (plan ?plan-id) (plan-order-class ?plan-order-class&:(>= ?ordering-class-selected  (- ?plan-order-class ?relaxation))))
    (not (pddl-action-precondition (plan ?plan-id) (id ?action-id) (context AGENDA-LOOKAHEAD)))
    (not (agenda-action-item (plan ?plan-id) (action ?action-id)))
    =>
    (assert (pddl-action-precondition (plan ?plan-id) (id ?action-id) (state PENDING) (context AGENDA-LOOKAHEAD)))
    (printout red "Checking action " ?action-id "(ordering " ?plan-order-class ") for preconditions, relaxed agenda candidate" crlf)
)

(defrule agenda-add-action-relaxed-reset
  (agenda (plan ?plan-id) (class-selection ?ordering-class-selected) (class-relaxation ?relaxation))
  (pddl-action (id ?action-id) (plan ?plan-id) (plan-order-class ?plan-order-class&:(>= ?ordering-class-selected  (- ?plan-order-class ?relaxation))))
  (not (agenda-action-item (plan ?plan-id) (action ?action-id)))
  (agenda-action-item (plan ?plan-id) (action ?completed-action-id&:(neq ?completed-action-id ?action-id)) (execution-state EFFECTS-APPLIED))
  ?precon <- (pddl-action-precondition (plan ?plan-id) (id ?action-id) (context AGENDA-LOOKAHEAD) (state PRECONDITION-UNSAT|PRECONDITION-SAT|PENDING))
  =>
  (retract ?precon)
)

(defrule agenda-action-check-start
  (agenda-action-item (plan ?plan-id) (action ?action-id) (execution-state PENDING))
  (not (pddl-action-precondition (plan ?plan-id) (id ?action-id) (context AGENDA-SELECTION)))
  =>
  (assert (pddl-action-precondition (plan ?plan-id) (id ?action-id) (state PENDING) (context AGENDA-SELECTION)))
)

(defrule agenda-action-check-reset
  (agenda-action-item (plan ?plan-id) (action ?action-id) (execution-state PENDING))
  ?precon <- (pddl-action-precondition (plan ?plan-id) (id ?action-id) (state PRECONDITION-UNSAT|PRECONDITION-SAT|PENDING) (context AGENDA-SELECTION))
  (agenda-action-item (plan ?plan-id) (action ?completed-action-id&:(neq ?completed-action-id ?action-id)) (execution-state EFFECTS-APPLIED))
  =>
  (retract ?precon)
)

(defrule agenda-action-print-sat
  ;(agenda-action-item (plan ?plan-id) (action ?action-id) (execution-state PENDING))
  (pddl-action-precondition (plan ?plan-id) (id ?action-id) (state PRECONDITION-SAT) (context ?context))
  =>
  (printout green "Action " ?action-id " has satisfied preconditions for context " ?context crlf)
)

(defrule agenda-action-print-unsat
  ;(agenda-action-item (plan ?plan-id) (action ?action-id) (execution-state PENDING))
  (pddl-action-precondition (plan ?plan-id) (id ?action-id) (state PRECONDITION-UNSAT) (context ?context) (unsatisfied-preconditions $?unsats))
  =>
  (printout yellow "Action " ?action-id " has unsatisfied preconditions: " ?unsats " for context " ?context crlf)
)
