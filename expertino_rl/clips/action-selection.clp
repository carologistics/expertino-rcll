(defrule agenda-select-first-plan
  ;(not (pddl-plan (state EXECUTING)))
  (pddl-plan (id ?plan-id) (state PLANNING))
  (not (agenda (plan ?plan-id)))
  =>
  (assert (agenda (plan ?plan-id) (state ACTIVE)))
  (printout green "Initialiasing new action agenda from plan " ?plan-id crlf)
)

(defrule rl-action-selected-spawn-and-transport
    (rl-action (id ?action-id) (is-selected TRUE))
    ?a <- (pddl-action (name spawn-and-transport) (id ?action-id))
    (not (agenda-action-item (action ?action-id)))
    (agenda (plan ?plan-id) (state ACTIVE))
    =>
    (assert (agenda-action-item (action ?action-id) (plan ?plan-id) (priority 0 1) (worker-type AGENT) (execution-state SELECTED) (worker AGENT)))
)

(defrule rl-action-selected-transport
    (rl-action (id ?action-id) (is-selected TRUE))
    ?a <- (pddl-action (name transport) (id ?action-id))
    (not (agenda-action-item (action ?action-id)))
    (agenda (plan ?plan-id) (state ACTIVE))

    ;assign the worker of the right type with the longest waiting time
    (worker (id ?worker) (state IDLE) (type ROBOT))
    (worker-idle-timer (worker ?worker) (start-time ?start-time))
    (not 
        (and
            (worker-idle-timer (worker ?other-worker) (start-time ?other-start-time&:(< ?other-start-time ?start-time)))
            (worker (id ?other-worker) (type ROBOT))
        )
    )
    =>
    (assert (agenda-action-item (action ?action-id) (plan ?plan-id) (priority 0 1) (worker-type ROBOT) (execution-state SELECTED) (worker ?worker)))
    (printout green "Assigned worker  " ?worker " of type  ROBOT, waiting since " ?start-time crlf)
)

(defrule rl-action-selected-base-transport
    (rl-action (id ?action-id) (is-selected TRUE))
    ?a <- (pddl-action (name base-transport) (id ?action-id))
    (not (agenda-action-item (action ?action-id)))
    (agenda (plan ?plan-id) (state ACTIVE))

    ;assign the worker of the right type with the longest waiting time
    (worker (id ?worker) (state IDLE) (type ROBOT))
    (worker-idle-timer (worker ?worker) (start-time ?start-time))
    (not 
        (and
            (worker-idle-timer (worker ?other-worker) (start-time ?other-start-time&:(< ?other-start-time ?start-time)))
            (worker (id ?other-worker) (type ROBOT))
        )
    )
    =>
    (assert (agenda-action-item (action ?action-id) (plan ?plan-id) (priority 0 1) (worker-type ROBOT) (execution-state SELECTED) (worker ?worker)))
    (printout green "Assigned worker  " ?worker " of type ROBOT, waiting since " ?start-time crlf)
)

(defrule rl-action-selected-pay-with-carrier
    (rl-action (id ?action-id) (is-selected TRUE))
    ?a <- (pddl-action (name pay-with-carrier) (id ?action-id))
    (not (agenda-action-item (action ?action-id)))
    (agenda (plan ?plan-id) (state ACTIVE))

    ;assign the worker of the right type with the longest waiting time
    (worker (id ?worker) (state IDLE) (type ROBOT))
    (worker-idle-timer (worker ?worker) (start-time ?start-time))
    (not 
        (and
            (worker-idle-timer (worker ?other-worker) (start-time ?other-start-time&:(< ?other-start-time ?start-time)))
            (worker (id ?other-worker) (type ROBOT))
        )
    )
    =>
    (assert (agenda-action-item (action ?action-id) (plan ?plan-id) (priority 0 1) (worker-type ROBOT) (execution-state SELECTED) (worker ?worker)))
    (printout green "Assigned worker  " ?worker " of type ROBOT, waiting since " ?start-time crlf)
)

(defrule rl-action-selected-carrier-to-input
    (rl-action (id ?action-id) (is-selected TRUE))
    ?a <- (pddl-action (name carrier-to-input) (id ?action-id))
    (not (agenda-action-item (action ?action-id)))
    (agenda (plan ?plan-id) (state ACTIVE))

    ;assign the worker of the right type with the longest waiting time
    (worker (id ?worker) (state IDLE) (type ROBOT))
    (worker-idle-timer (worker ?worker) (start-time ?start-time))
    (not 
        (and
            (worker-idle-timer (worker ?other-worker) (start-time ?other-start-time&:(< ?other-start-time ?start-time)))
            (worker (id ?other-worker) (type ROBOT))
        )
    )
    =>
    (assert (agenda-action-item (action ?action-id) (plan ?plan-id) (priority 0 1) (worker-type ROBOT) (execution-state SELECTED) (worker ?worker)))
    (printout green "Assigned worker  " ?worker " of type ROBOT, waiting since " ?start-time crlf)
)

(defrule rl-action-selected-pay-from-bs
    (rl-action (id ?action-id) (is-selected TRUE))
    ?a <- (pddl-action (name pay-from-bs) (id ?action-id))
    (not (agenda-action-item (action ?action-id)))
    (agenda (plan ?plan-id) (state ACTIVE))
    =>
    (assert (agenda-action-item (action ?action-id) (plan ?plan-id) (priority 0 1) (worker-type AGENT) (execution-state SELECTED) (worker AGENT)))
)

(defrule rl-action-selected-transport-to-slide
    (rl-action (id ?action-id) (is-selected TRUE))
    ?a <- (pddl-action (name transport-to-slide) (id ?action-id))
    (not (agenda-action-item (action ?action-id)))
    (agenda (plan ?plan-id) (state ACTIVE))

    ;assign the worker of the right type with the longest waiting time
    (worker (id ?worker) (state IDLE) (type ROBOT))
    (worker-idle-timer (worker ?worker) (start-time ?start-time))
    (not 
        (and
            (worker-idle-timer (worker ?other-worker) (start-time ?other-start-time&:(< ?other-start-time ?start-time)))
            (worker (id ?other-worker) (type ROBOT))
        )
    )
    =>
    (assert (agenda-action-item (action ?action-id) (plan ?plan-id) (priority 0 1) (worker-type ROBOT) (execution-state SELECTED) (worker ?worker)))
    (printout green "Assigned worker  " ?worker " of type ROBOT, waiting since " ?start-time crlf)
)

(defrule rl-action-agent-expand
    (agenda-action-item (action ?action-id) (worker-type AGENT) (worker AGENT) (execution-state EXECUTING))
    (executor (id ?ex-id) (pddl-action-id ?action-id) (worker AGENT))
    (pddl-plan (id ?plan-id) (context ?ex-id))
    (not (agenda-action-item (plan ?plan-id)))
    ?d <- (delay-executability-check)

    ;assign the worker of the right type with the longest waiting time
    (worker (id ?worker) (state IDLE) (type ROBOT))
    (worker-idle-timer (worker ?worker) (start-time ?start-time))
    (not 
        (and
            (worker-idle-timer (worker ?other-worker) (start-time ?other-start-time&:(< ?other-start-time ?start-time)))
            (worker (id ?other-worker) (type ROBOT))
        )
    )
    (confval (path "/pddl/actions/refbox") (list-value $?refbox-actions))
    =>
    (do-for-all-facts ((?a pddl-action))
        (eq ?a:plan ?plan-id)
        (if (member$ (str-cat ?a:name) ?refbox-actions) then
            (assert (agenda-action-item (action ?a:id) (plan ?a:plan) (priority 0 1) (worker-type REFBOX) (worker REFBOX) (execution-state SELECTED)))
        else
            (assert (agenda-action-item (action ?a:id) (plan ?a:plan) (priority 0 1) (worker-type ROBOT) (worker ?worker) (execution-state PENDING)))
            (printout green "Assigned worker  " ?worker " of type ROBOT, waiting since " ?start-time crlf)
        )
    )
    (retract ?d)
)

(defrule rl-action-agent-select-robot-action
    (agenda-action-item (action ?action-id) (plan ?plan) (worker-type REFBOX) (execution-state COMPLETED))
    ;(not (agenda-action-item (action ?action-id2&~?action-id) (plan ?plan) (worker-type REFBOX) (worker REFBOX) (execution-state ?state&~COMPLETED)))
    ?aai <- (agenda-action-item (action ?robot-action) (plan ?plan) (worker-type ROBOT) (worker ?worker) (execution-state PENDING))
    =>
    (modify ?aai (execution-state SELECTED))
)

(defrule agenda-complete
  (declare (salience -1))
  ?agenda <- (agenda (plan ?plan-id) (class-selection ?ordering-class) (state ACTIVE))
  ?plan <- (pddl-plan (id ?plan-id) (instance ?instance))
  (pddl-instance (name ?instance))
  (forall   (pddl-action (id ?action-id) (plan ?plan-id))
            (agenda-action-item (action ?action-id) (execution-state COMPLETED)))
  =>
  (printout green "Agenda for plan" ?plan-id "has been completed" crlf)
  (retract ?agenda)
  (do-for-all-facts ((?action pddl-action)) (eq pddl-action:plan ?plan-id)
    (retract ?action)
  )
  (retract ?plan)
) 