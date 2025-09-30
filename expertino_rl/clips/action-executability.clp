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

(defrule check-action
    (declare (salience ?*SALIENCE-ACTION-EXECUTABLE-CHECK*))
    (rl-executability-check (state CHECKING))
    (pddl-action (id ?action-id) (name ?action))
    (not (pddl-action-precondition (id ?action-id)))
    (not (rl-action (id ?action-id) (is-finished TRUE)))
    (not (rl-action (name ?action) (is-selected TRUE)))
    =>
    (assert (pddl-action-precondition (id ?action-id)))
)

(defrule executable-action
    (declare (salience ?*SALIENCE-ACTION-EXECUTABLE-CHECK*))
    (rl-executability-check (state CHECKING))
    (pddl-action-precondition (id ?action-id) (state PRECONDITION-SAT))
    (pddl-action (id ?action-id) (name ?name) (params ?params))
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
        (bind ?points (first$ $?mapping))
        (bind ?action-params (rest$ $?mapping))
        
        (assert (rl-action (id ?action-id) (name (sym-cat ?name "#" (create-slot-value-string ?action-params))) (points ?points)))
    else
        (assert (agenda-action-item (action ?action-id) (plan (gensym*)) (priority 0 1) (worker-type ?worker-type)) (execution-state SELECTED))
    )
)

(defrule executability-check-finished
    (declare (salience (- ?*SALIENCE-ACTION-EXECUTABLE-CHECK* 1)))
    ?ec <- (rl-executability-check (state CHECKING))
    (not (pddl-action-precondition (state PENDING|CHECK-PRECONDITION)))
    =>
    (modify ?ec (state CHECKED))
    (do-for-all-facts ((?ap pddl-action-precondition))
        (retract ?ap)
    )
)
