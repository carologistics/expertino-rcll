(deffunction rl-action-map-params-and-points (?id ?name ?params)
    (switch ?name
        (case spawn-and-transport then
            (bind ?param-indices (create$ 1))
            (bind ?points ?*POINTS-ACTION-SPAWN-AND-TRANSPORT*))
        (case transport then
            (bind ?param-indices (create$ 1))
            (bind ?points ?*POINTS-ACTION-TRANSPORT*))
        (case base-transport then
            (bind ?param-indices (create$ 1))
            (bind ?points ?*POINTS-ACTION-BASE-TRANSPORT*))
        (case pay-with-carrier then
            (bind ?param-indices (create$ 1 2))
            (bind ?points ?*POINTS-ACTION-PAY-WITH-CARRIER*))
        (case carrier-to-input then
            (bind ?param-indices (create$ 1 3))
            (bind ?points ?*POINTS-ACTION-CARRIER-TO-INPUT*))
        (case pay-from-bs then
            (bind ?param-indices (create$ 1))
            (bind ?points ?*POINTS-ACTION-PAY-FROM-BS*))
        (case transport-to-slide then
            (bind ?param-indices (create$ 1 4))
            (bind ?points ?*POINTS-ACTION-TRANSPORT-TO-SLIDE*))
        (default (bind ?param-indices (create$)))
    )
    (bind ?action-params (create$))
    (foreach ?index ?param-indices
        (bind ?param (nth$ ?index ?params))
        (if (member$ ?name (create$ spawn-and-transport transport base-transport)) then
            (bind ?prod-name-end (- (str-index "-gen" (str-cat ?param)) 1))
            (bind ?param (sym-cat (sub-string 1 ?prod-name-end (str-cat ?param))))
        )
        (bind ?action-params (create$ ?action-params ?param))
    )
    (return (create$ ?points ?action-params))
    
)

(defrule delay-action-space
    (declare (salience 1))
    (agenda-action-item (action ?action-id) (worker-type AGENT) (worker AGENT) (execution-state SELECTED))
    (not (executor (id ?ex-id) (pddl-action-id ?action-id) (worker AGENT)))
    =>
    (assert (delay-action-space))
)

(defrule check-action
    (rl-current-action-space (state PENDING))
    (pddl-action (id ?action-id) (name ?name))
    (not (pddl-action-condition (action ?action-id)))
    (not (agenda-action-item (action ?action-id)))
    (not (delay-executability-check))

    (confval (path "/pddl/actions/robot") (list-value $?robot-actions))
    (confval (path "/pddl/actions/refbox") (list-value $?refbox-actions))
    (confval (path "/pddl/actions/agent") (list-value $?agent-actions))

    (test (member$ (str-cat ?name) (create$ ?robot-actions ?refbox-actions ?agent-actions)))

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
    (confval (path "/pddl/actions/refbox") (list-value $?refbox-actions))
    (confval (path "/pddl/actions/agent") (list-value $?agent-actions))
    =>
    (bind ?worker-type (if (member$ (str-cat ?name) ?robot-actions)
                       then ROBOT
                       else 
                         (if (member$ (str-cat ?name) ?refbox-actions)
                          then REFBOX
                          else AGENT)))
    (if (or (eq ?worker-type ROBOT) (eq ?worker-type AGENT)) then
        (bind $?mapping (rl-action-map-params-and-points ?action-id ?name ?params))
        (bind ?points (nth$ 1 $?mapping))
        (bind ?action-params (rest$ $?mapping))
        
        (assert (rl-action (id ?action-id) (name ?name) (params ?action-params) (reward ?points)))
    else
        (assert (agenda-action-item (action ?action-id) (plan ?plan-id) (priority 0 1) (worker-type ?worker-type) (worker REFBOX) (execution-state SELECTED)))
        (printout green "Executing REFBOX action " ?name ?params crlf)
    )
)

(defrule executability-check-finished
    ?ca <- (rl-current-action-space (state PENDING))
    (not (pddl-action-condition (state PENDING|CHECK-CONDITION)))
    (not (delay-action-space))
    =>
    (modify ?ca (state DONE))
    ;(retract ?ec)
    ;(assert (rl-executability-check (state CHECKED)))
    (do-for-all-facts ((?ap pddl-action-condition))
        (retract ?ap)
    )
)
