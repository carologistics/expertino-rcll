(defrule execute-action
    (rl-action (id ?action-id) (is-selected TRUE))
    (pddl-action (id ?action-id))
    (not (pddl-action-apply-effect (action ?action-id)))
    =>
    (assert (pddl-action-apply-effect (action ?action-id)))
)

(defrule execute-action-done
    (pddl-action-apply-effect (action ?action-id) (state DONE))
    ?ra <- (rl-action (id ?action-id) (is-selected TRUE))
    =>
    (modify ?ra (is-finished TRUE))
)