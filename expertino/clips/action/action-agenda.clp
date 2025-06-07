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
  (printout green "Initialiasing action agenda from plan " ?plan-id crlf)
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
  (printout green "Agenda moved ahead to class " (+ ?ordering-class 1) crlf)
)

(defrule agenda-add-action
  (agenda (plan ?plan-id) (class-selection ?ordering-class) (class-relaxation ?relaxation))
  (pddl-action (id ?action-id) (name ?action-name) (plan ?plan-id) (plan-order-class ?ordering-class))
  (not (agenda-action-item (plan ?plan-id) (action ?action-id)))
  (confval (path "/pddl/actions/robot") (list-value $?robot-actions))
  (confval (path "/pddl/actions/refbox") (list-value $?refbox-actions))
  (confval (path "/pddl/actions/agent") (list-value $?agent-actions))
  =>
  (bind ?worker-type (if (member$ (str-cat ?action-name) ?robot-actions)
                       then ROBOT
                       else 
                         (if (member$ (str-cat ?action-name) ?refbox-actions)
                          then REFBOX
                          else AGENT)))
  (assert (agenda-action-item (action ?action-id) (plan ?plan-id) (priority 0 1) (worker-type ?worker-type)))
  (printout yellow "Added action " ?action-id "(ordering " ?ordering-class ") to agenda" crlf)
)

(defrule agenda-add-action-relaxed
  (agenda (plan ?plan-id) (class-selection ?ordering-class-selected) (class-relaxation ?relaxation))
  (pddl-action (id ?action-id) (name ?action-name) (plan ?plan-id) (plan-order-class ?plan-order-class&:(>= ?ordering-class-selected  (- ?plan-order-class ?relaxation))))
  ?precon <- (pddl-action-precondition (plan ?plan-id) (id ?action-id) (state PRECONDITION-SAT) (context AGENDA-LOOKAHEAD))
  (not (agenda-action-item (plan ?plan-id) (action ?action-id)))
  (confval (path "/pddl/actions/robot") (list-value $?robot-actions))
  (confval (path "/pddl/actions/refbox") (list-value $?refbox-actions))
  (confval (path "/pddl/actions/agent") (list-value $?agent-actions))
  =>
  (bind ?worker-type (if (member$ (str-cat ?action-name) ?robot-actions)
                       then ROBOT
                       else 
                         (if (member$ (str-cat ?action-name) ?refbox-actions)
                          then REFBOX
                          else AGENT)))
  (assert (agenda-action-item (action ?action-id) (plan ?plan-id) (priority 0 0) (worker-type ?worker-type)))
  (retract ?precon)
  (printout yellow "Added action " ?action-id "(" ?plan-order-class ") to agenda (relaxed condition, precondition satisfied)" crlf)
)

(defrule agenda-add-action-relaxed-check
    (agenda (plan ?plan-id) (class-selection ?ordering-class-selected) (class-relaxation ?relaxation))
    (pddl-action (id ?action-id) (plan ?plan-id) (plan-order-class ?plan-order-class&:(>= ?ordering-class-selected  (- ?plan-order-class ?relaxation))))
    (not (pddl-action-precondition (plan ?plan-id) (id ?action-id) (context AGENDA-LOOKAHEAD)))
    (not (agenda-action-item (plan ?plan-id) (action ?action-id)))
    (pddl-plan (id ?plan-id) (instance ?instance))
    (pddl-instance-update (instance ?instance) (last-updated ?last-update-time))
    =>
    (assert (pddl-action-precondition (instance ?instance) (plan ?plan-id) (id ?action-id) (state PENDING) (context AGENDA-LOOKAHEAD) (instance-update ?last-update-time)))
    (printout yellow "Checking action " ?action-id "(ordering " ?plan-order-class ") for preconditions, relaxed agenda candidate" crlf)
)

(defrule agenda-add-action-relaxed-reset
  (agenda (plan ?plan-id) (class-selection ?ordering-class-selected) (class-relaxation ?relaxation))
  (pddl-action (id ?action-id) (plan ?plan-id) (plan-order-class ?plan-order-class&:(>= ?ordering-class-selected  (- ?plan-order-class ?relaxation))))
  ?precon <- (pddl-action-precondition (plan ?plan-id) (id ?action-id) (context AGENDA-LOOKAHEAD) (state PRECONDITION-UNSAT|PRECONDITION-SAT|PENDING) (instance-update ?update-time))
  (pddl-instance-update (instance ?instance) (last-updated ?last-update-time&:(> ?last-update-time ?update-time)))
  =>
  (retract ?precon)
)
