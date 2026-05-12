(defrule rl-action-selected-robot
    (rl-action (id ?action-id) (is-selected TRUE))
    ?a <- (pddl-action (name ?name) (id ?action-id))
    (not (executor (action-id ?action-id)))
    (confval (path "/pddl/actions/robot") (list-value $?robot-actions))
    (test (member$ (str-cat ?name) ?robot-actions))
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
    (assert (executor (id (sym-cat EXECUTOR-(gensym*))) (action-id ?action-id) (state INIT) (worker ?worker)))
    (printout green "Assigned worker  " ?worker " of type  ROBOT, waiting since " ?start-time crlf)
)

(defrule rl-action-selected-refbox
    (rl-action (id ?action-id) (is-selected TRUE))
    ?a <- (pddl-action (name ?name) (id ?action-id))
    (not (executor (action-id ?action-id)))
    (confval (path "/pddl/action/refbox_select") (list-value $?refbox-select-actions))
    (test (member$ (str-cat ?name) ?refbox-select-actions))
    =>
    (assert (executor (id (sym-cat EXECUTOR-(gensym*))) (action-id ?action-id) (state INIT) (worker ?worker)))
    (printout green "Executing REFBOX action " ?name ?params crlf)
)
