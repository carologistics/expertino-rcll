(defrule rl-action-finished
    (agenda-action-item (action ?action-id) (execution-state COMPLETED))
    ?r <- (rl-action (id ?action-id) (is-selected TRUE) (is-finished FALSE))
    =>
    (modify ?r (is-finished TRUE))
)

(defrule rl-action-finished
    (agenda-action-item (action ?action-id) (execution-state ERROR))
    ?r <- (rl-action (id ?action-id) (is-selected TRUE) (is-finished FALSE))
    =>
    (modify ?r (is-finished TRUE) (points 0))
)