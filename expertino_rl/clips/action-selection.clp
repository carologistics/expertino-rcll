(defrule rl-action-selected-spawn-and-transport
    (rl-action (name ?name) (is-selected TRUE))
    (test (str-index "spawn-and-transport" (str-cat ?name)))
    (pddl-fluent (name step) (params ?prod ?step))
    (test (str-index (sub-string 1 (- (str-index "-gen" (str-cat ?prod)) 1) (str-cat ?prod)) (str-cat ?name)))
    (pddl-fluent (name next-step) (params ?prod ?step ?next))
    (pddl-fluent (name step-place) (params ?next ?to))
    ?a <- (pddl-action (name spawn-and-transport) (id ?action-id) (params ?prod ?to ?step ?next))
    =>
    (assert (agenda-action-item (action ?action-id) (plan (gensym*)) (priority 0 1) (worker-type AGENT) (execution-state SELECTED)))
)

(defrule rl-action-selected-transport
    (rl-action (name ?name) (is-selected TRUE))
    (test (str-index "transport" (str-cat ?name)))
    (not (or    (test (str-index "spawn-and-transport" (str-cat ?name)))
                (test (str-index "transport-to-slide" (str-cat ?name)))
                (test (str-index "base-transport" (str-cat ?name)))))
    (pddl-fluent (name step) (params ?prod ?step))
    (test (str-index (sub-string 1 (- (str-index "-gen" (str-cat ?prod)) 1) (str-cat ?prod)) (str-cat ?name)))
    (pddl-fluent (name at) (params ?prod ?from))
    (pddl-fluent (name step-place) (params ?step ?to))
    ?a <- (pddl-action (name transport) (id ?action-id) (params ?prod ?from ?to ?step))
    =>
    (assert (agenda-action-item (action ?action-id) (plan (gensym*)) (priority 0 1) (worker-type ROBOT) (execution-state SELECTED)))
)

(defrule rl-action-selected-base-transport
    (rl-action (name ?name) (is-selected TRUE))
    (test (str-index "base-transport" (str-cat ?name)))
    (pddl-fluent (name step) (params ?prod ?step))
    (test (str-index (sub-string 1 (- (str-index "-gen" (str-cat ?prod)) 1) (str-cat ?prod)) (str-cat ?name)))
    (pddl-fluent (name at) (params ?prod ?from))
    (pddl-fluent (name next-step) (params ?prod ?step ?next))
    (pddl-fluent (name step-place) (params ?next ?to))
    ?a <- (pddl-action (name base-transport) (id ?action-id) (params ?prod ?from ?to ?step ?next))
    =>
    (assert (agenda-action-item (action ?action-id) (plan (gensym*)) (priority 0 1) (worker-type ROBOT) (execution-state SELECTED)))
)

(defrule rl-action-selected-pay-with-carrier
    (rl-action (name ?name) (is-selected TRUE))
    (test (str-index "pay-with-carrier" (str-cat ?name)))
    (pddl-fluent (name rs-slide) (params ?m ?to))
    (test (str-index (str-cat ?m) (str-cat ?name)))
    (pddl-fluent (name at) (params ?carrier ?from))
    (test (str-index (str-cat ?carrier) (str-cat ?name)))
    ?a <- (pddl-action (name pay-with-carrier) (id ?action-id) (params ?m ?carrier ?from ?to))
    =>
    (assert (agenda-action-item (action ?action-id) (plan (gensym*)) (priority 0 1) (worker-type ROBOT) (execution-state SELECTED)))
)

(defrule rl-action-selected-carrier-to-input
    (rl-action (name ?name) (is-selected TRUE))
    (test (str-index "carrier-to-input" (str-cat ?name)))
    (pddl-fluent (name in) (params ?cs ?in))
    (test (str-index (str-cat ?cs) (str-cat ?name)))
    (pddl-fluent (name token-step) (params ?carrier ?in ?step))
    (test (str-index (str-cat ?carrier) (str-cat ?name)))
    ?a <- (pddl-action (name carrier-to-input) (id ?action-id) (params ?carrier ?step ?cs ?in))
    =>
    (assert (agenda-action-item (action ?action-id) (plan (gensym*)) (priority 0 1) (worker-type ROBOT) (execution-state SELECTED)))
)

(defrule rl-action-selected-pay-from-bs
    (rl-action (name ?name) (is-selected TRUE))
    (test (str-index "pay-from-bs" (str-cat ?name)))
    (pddl-fluent (name rs-slide) (params ?rs ?slide))
    (test (str-index (str-cat ?rs) (str-cat ?name)))
    ?a <- (pddl-action (name pay-from-bs) (id ?action-id) (params ?rs))
    =>
    (assert (agenda-action-item (action ?action-id) (plan (gensym*)) (priority 0 1) (worker-type AGENT) (execution-state SELECTED)))
)

(defrule rl-action-selected-transport-to-slide
    (rl-action (name ?name) (is-selected TRUE))
    (test (str-index "transport-to-slide" (str-cat ?name)))
    (pddl-fluent (name at) (params ?token ?from))
    (test (str-index (str-cat ?token) (str-cat ?name)))
    (pddl-fluent (name rs-slide) (params ?rs ?to))
    (test (str-index (str-cat ?rs) (str-cat ?name)))
    ?a <- (pddl-action (name transport-to-slide) (id ?action-id) (params ?token ?from ?to ?rs))
    =>
    (assert (agenda-action-item (action ?action-id) (plan (gensym*)) (priority 0 1) (worker-type ROBOT)) (execution-state SELECTED))
)

