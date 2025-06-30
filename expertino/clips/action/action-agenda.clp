; The agenda module manages the set of actions currently ready for execution or in progress.
; An agenda is created from a plan that was selected for execution and is updated based on the
; current progress within the partial order of the plan. The agenda is used to determine the
; next actions to execute and to check the preconditions of actions that are ready for execution
; before assigning them to executors. 

(defrule agenda-select-first-plan
  ;(not (pddl-plan (state EXECUTING)))
  ?plan <- (pddl-plan (id ?plan-id) (state SELECTED))
  (not (agenda (plan ?plan-id)))
  =>
  (assert (agenda (plan ?plan-id) (state ACTIVE)))
  (modify ?plan (state EXECUTING))
  (printout green "Initialiasing new action agenda from plan " ?plan-id crlf)
)

(defrule agenda-step-ahead
  ?ag <- (agenda (plan ?plan-id) (class-selection ?ordering-class) (state ACTIVE))
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
  (agenda (plan ?plan-id) (class-selection ?ordering-class) (class-relaxation ?relaxation) (state ACTIVE))
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
  (agenda (plan ?plan-id) (class-selection ?ordering-class-selected) (class-relaxation ?relaxation) (state ACTIVE))
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
    (agenda (plan ?plan-id) (class-selection ?ordering-class-selected) (class-relaxation ?relaxation) (state ACTIVE))
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
  (agenda (plan ?plan-id) (class-selection ?ordering-class-selected) (class-relaxation ?relaxation) (state ACTIVE))
  (pddl-action (id ?action-id) (plan ?plan-id) (plan-order-class ?plan-order-class&:(>= ?ordering-class-selected  (- ?plan-order-class ?relaxation))))
  ?precon <- (pddl-action-precondition (plan ?plan-id) (id ?action-id) (context AGENDA-LOOKAHEAD) (state PRECONDITION-UNSAT|PRECONDITION-SAT|PENDING) (instance-update ?update-time))
  (pddl-instance-update (instance ?instance) (last-updated ?last-update-time&:(> ?last-update-time ?update-time)))
  =>
  (retract ?precon)
)

(defrule agenda-freeze
  (declare (salience ?*SALIENCE-FREEZE-AGENDA*))
  (freeze-agenda (instance ?instance))
  (pddl-plan (id ?plan-id) (instance ?instance))
  ?agenda <- (agenda (plan ?plan-id) (state ACTIVE)) 
  =>
  (printout debug "Freezing agenda.." crlf)
  (modify ?agenda (state INACTIVE))
  ;retract all pending precondition-check facts
  (do-for-all-facts ((?precon pddl-action-precondition)) (and (eq ?precon:plan ?plan-id) (eq ?precon:state PENDING))
    (retract ?precon)
  )
  ;retract all the executors for the agenda that are in state INIT
  (do-for-all-facts ((?ex executor) (?ai agenda-action-item))
    (and (eq ?ai:plan ?plan-id) (eq ?ex:pddl-action-id ?ai:action) (eq ?ex:state INIT))
    (retract ?ex)
  )
)

(defrule agenda-frozen
  (declare (salience ?*SALIENCE-FREEZE-AGENDA*))
  ?fa <- (freeze-agenda (instance ?instance))
  (pddl-instance (name ?instance) (busy-with FALSE))
  (not (and 
         (pddl-plan (id ?plan-id) (instance ?instance))
         (or
           (agenda (plan ?plan-id) (state ACTIVE))
           (agenda-action-item (plan ?plan-id) (execution-state EXECUTING))
           (pending-pddl-fluent (instance ?instance))
           (pending-pddl-numeric-fluent (instance ?instance))
           (pending-pddl-object (instance ?instance))
           (pddl-action-get-effect (state ~DONE))
         )
       )
  )
  =>
  (retract ?fa) 
)

(defrule agenda-complete
  ?agenda <- (agenda (plan ?plan-id) (class-selection ?ordering-class) (state ACTIVE))
  ?plan <- (pddl-plan (id ?plan-id) (instance ?instance))
  (pddl-instance (name ?instance))
  (not (pddl-action (id ?action-id) (plan ?plan-id) 
            (plan-order-class ?ordering-class-o&:(> ?ordering-class-o ?ordering-class))))
  (not (agenda-action-item (plan ?plan-id) (execution-state ~COMPLETED)))
  =>
  (printout green "Agenda for plan" ?plan-id "has been completed" crlf)
  (retract ?agenda)
  (do-for-all-facts ((?action pddl-action)) (eq pddl-action:plan ?plan-id)
    (retract ?action)
  )
  (retract ?plan)
) 
