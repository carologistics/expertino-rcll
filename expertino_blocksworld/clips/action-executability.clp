(defrule check-action
    (declare (salience ?*SALIENCE-ACTION-EXECUTABLE-CHECK*))
    (rl-executability-check (state CHECKING))
    (pddl-action (id ?action-id) (name ?action))
    (not (pddl-action-condition (action ?action-id)))
    (not (rl-action (id ?action-id) (is-finished TRUE)))
    (not (rl-action (name ?action) (is-selected TRUE)))
    =>
    (assert (pddl-action-condition (action ?action-id)))
)

(defrule executable-action
    (declare (salience ?*SALIENCE-ACTION-EXECUTABLE-CHECK*))
    (rl-executability-check (state CHECKING))
    (pddl-action-condition (action ?action-id) (state CONDITION-SAT))
    (pddl-action (id ?action-id) (name ?name) (params $?params))
    =>
    (assert (rl-action (id ?action-id) (name (sym-cat ?name "#" (create-slot-value-string $?params))) (points 0)))
)

(defrule executability-check-finished
    (declare (salience (- ?*SALIENCE-ACTION-EXECUTABLE-CHECK* 1)))
    ?ec <- (rl-executability-check (state CHECKING))
    (not (pddl-action-condition (state PENDING|CHECK-CONDITION)))
    =>
    (modify ?ec (state CHECKED))
    (do-for-all-facts ((?ap pddl-action-condition))
        (retract ?ap)
    )
)

