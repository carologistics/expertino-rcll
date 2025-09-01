(defrule execute-action
    (rl-action (id ?action-id) (is-selected TRUE))
    (pddl-action (id ?action-id))
    (not (pddl-action-get-effect (action ?action-id)))
    =>
    (assert (pddl-action-get-effect (action ?action-id) (apply TRUE)))
)

(defrule execute-action-done
    (pddl-action-get-effect (action ?action-id) (state DONE))
    ?ra <- (rl-action (id ?action-id) (is-selected TRUE))
	(rl-mode (mode ?mode))
	?rec <- (rl-executability-check (state ?state))
    =>
    (modify ?ra (is-finished TRUE))
	(if (eq ?mode EXECUTION) then
	  (modify ?rec (state PENDING))
	)
)
