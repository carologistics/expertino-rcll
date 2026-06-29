(deffunction rl-action-map-params-and-points (?id ?name ?params)
    (switch ?name
        (case transport-to-cs then
            (bind ?param-indices (create$ 1))
            (bind ?points ?*POINTS-ACTION-TRANSPORT-TO-CS*))
        (case transport then
            (bind ?param-indices (create$ 1))
            (bind ?points ?*POINTS-ACTION-TRANSPORT*))
        (case pay-with-base then
            (bind ?param-indices (create$ 1 3))
            (bind ?points ?*POINTS-ACTION-PAY-WITH-BASE*))
        (case pay-with-carrier then
            (bind ?param-indices (create$ 1 3))
            (bind ?points ?*POINTS-ACTION-PAY-WITH-CARRIER*))
        (case carrier-to-input then
            (bind ?param-indices (create$ 1 3))
            (bind ?points ?*POINTS-ACTION-CARRIER-TO-INPUT*))
        (case bs-dispense then
            (bind ?param-indices (create$ 1))
            (bind ?points ?*POINTS-ACTION-BS-DISPENSE*))
        (case bs-dispense-pay then
            (bind ?param-indices (create$ 1))
            (bind ?points ?*POINTS-ACTION-BS-DISPENSE-PAY*))
    )
    (bind ?action-params (create$))
    (foreach ?index ?param-indices
        (bind ?param (nth$ ?index ?params))
        (if (member$ ?name (create$ transport transport-to-cs bs-dispense)) then
            (bind ?prod-name-end (- (str-index "-gen" (str-cat ?param)) 1))
            (bind ?param (sym-cat (sub-string 1 ?prod-name-end (str-cat ?param))))
        )
        (bind ?action-params (create$ ?action-params ?param))
    )
    (return (create$ ?points ?action-params))
    
)

(defrule check-action
    (declare (salience 1))
    (rl-current-action-space (state PENDING))
    (pddl-action (id ?action-id) (name ?name))
    (not (pddl-action-condition (action ?action-id)))
    (not (executor (action-id ?action-id)))
    (not (pddl-action-get-effect (state WAITING)))
    (not (rl-action (id ?action-id) (is-selected TRUE)))
    (not (rl-action (id ?action-id) (is-finished TRUE)))

    (confval (path "/pddl/actions/robot") (list-value $?robot-actions))
    (confval (path "/pddl/actions/refbox_auto") (list-value $?refbox-auto-actions))
    (confval (path "/pddl/actions/refbox_select") (list-value $?refbox-select-actions))

    (test (member$ (str-cat ?name) (create$ ?robot-actions ?refbox-auto-actions ?refbox-select-actions)))

    ;(not (rl-action (id ?action-id) (is-finished TRUE)))
    ;(not (rl-action (id ?action-id) (is-selected TRUE)))
    =>
    (assert (pddl-action-condition (action ?action-id)))
)

(defrule executable-action
    (rl-current-action-space (state PENDING))
    (pddl-action-condition (action ?action-id) (state CONDITION-SAT))
    (pddl-action (id ?action-id)  (plan ?plan-id) (name ?name) (params $?params))
    
    (confval (path "/pddl/actions/robot") (list-value $?robot-actions))
    (confval (path "/pddl/actions/refbox_auto") (list-value $?refbox-auto-actions))
    (confval (path "/pddl/actions/refbox_select") (list-value $?refbox-select-actions))
    =>
    (if (member$ (str-cat ?name) (create$ ?robot-actions ?refbox-select-actions)) then
        (bind $?mapping (rl-action-map-params-and-points ?action-id ?name ?params))
        (bind ?points (nth$ 1 $?mapping))
        (bind ?action-params (rest$ $?mapping))
        
        (assert (rl-action (id ?action-id) (name ?name) (params ?action-params) (reward ?points)))
    else
        (assert (executor (id (sym-cat EXECUTOR-(gensym*))) (action-id ?action-id) (worker REFBOX) (state INIT)))
        (printout green "Executing REFBOX action " ?name ?params crlf)
    )
)

(defrule executability-check-filter-bs
    (declare (salience 1))
    (rl-current-action-space (state PENDING))
    (or
        (rl-action (name bs-dispense|bs-dispense-pay) (is-selected TRUE) (is-finished FALSE))
        (not (and
            (pddl-fluent (name free) (params bs-input))
            (pddl-fluent (name free) (params bs-output))
        ))
    )
    ?ra <- (rl-action (id ?action-id) (name bs-dispense|bs-dispense-pay) (is-selected FALSE))
    =>
    (printout warn "Base station action not allowed: " ?action-id crlf)
    (retract ?ra)
)


(defrule executability-check-finished
    ?ca <- (rl-current-action-space (state PENDING))
    (not (pddl-action-condition (state PENDING|CHECK-CONDITION)))
    (not (pddl-action-get-effect (state WAITING)))
    (rl-action (id ?action-id))
    =>
    (modify ?ca (state DONE))
    ;(retract ?ec)
    ;(assert (rl-executability-check (state CHECKED)))
    (do-for-all-facts ((?ap pddl-action-condition))
        (retract ?ap)
    )
)
