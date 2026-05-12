(defrule pddl-action-from-step-base-to-cap
    (pddl-fluent (name next-step) (params ?wp ?base-step ?cap-step))
    (test (str-index base ?base-step))
    (test (str-index cap ?cap-step))
    (pddl-fluent (name step-place) (params ?cap-step ?cap-place))
    (pddl-fluent (name in) (params ?cap-station ?cap-place))
    (confval (path "/pddl/problem_instance") (value ?instance-str))
    =>
    (bind ?instance (sym-cat ?instance-str))
    (assert (pddl-action (instance ?instance) (id (gensym*)) 
                         (name bs-dispense) (params ?wp bs-output ?base-step ?cap-step)))
    (assert (pddl-action (instance ?instance) (id (gensym*)) 
                         (name transport-to-cs) (params ?wp bs-output ?cap-station ?cap-place ?cap-step)))
)

(defrule pddl-action-from-step-base-to-ring
    (pddl-fluent (name next-step) (params ?wp ?base-step ?ring-step))
    (test (str-index base ?base-step))
    (test (str-index ring ?ring-step))
    (pddl-fluent (name step-place) (params ?ring-step ?ring-place))
    (confval (path "/pddl/problem_instance") (value ?instance-str))
    =>
    (bind ?instance (sym-cat ?instance-str))
    (assert (pddl-action (instance ?instance) (id (gensym*)) 
                         (name bs-dispense) (params ?wp bs-output ?base-step ?ring-step)))
    (assert (pddl-action (instance ?instance) (id (gensym*))
                         (name transport) (params ?wp bs-output ?ring-place ?ring-step)))
)

(defrule pddl-action-from-step-ring-to-ring
    (pddl-fluent (name next-step) (params ?wp ?ring-step ?next-ring-step))
    (test (str-index ring ?ring-step))
    (test (str-index ring ?next-ring-step))
    (pddl-fluent (name step-place) (params ?ring-step ?ring-input))
    (pddl-fluent (name in) (params ?ring-station ?ring-intput))
    (pddl-fluent (name out) (params ?ring-station ?ring-output))
    (pddl-fluent (name step-place) (params ?next-ring-step ?next-ring-place))
    (confval (path "/pddl/problem_instance") (value ?instance-str))
    =>
    (bind ?instance (sym-cat ?instance-str))
    (assert (pddl-action (instance ?instance) (id (gensym*))
                         (name rs-mount-ring) (params ?wp ?ring-station ?ring-input ?ring-output ?ring-step ?next-ring-step)))
    (assert (pddl-action (instance ?instance) (id (gensym*))
                         (name transport) (params ?wp ?ring-output ?next-ring-place ?next-ring-step)))
)

(defrule pddl-action-from-step-ring-to-cap
    (pddl-fluent (name next-step) (params ?wp ?ring-step ?cap-step))
    (test (str-index ring ?ring-step))
    (test (str-index cap ?cap-step))
    (pddl-fluent (name step-place) (params ?ring-step ?ring-input))
    (pddl-fluent (name in) (params ?ring-station ?ring-intput))
    (pddl-fluent (name out) (params ?ring-station ?ring-output))
    (pddl-fluent (name step-place) (params ?cap-step ?cap-place))
    (pddl-fluent (name in) (params ?cap-station ?cap-place))
    (confval (path "/pddl/problem_instance") (value ?instance-str))
    =>
    (bind ?instance (sym-cat ?instance-str))
    (assert (pddl-action (instance ?instance) (id (gensym*))
                         (name rs-mount-ring) (params ?wp ?ring-station ?ring-input ?ring-output ?ring-step ?cap-step)))
    (assert (pddl-action (instance ?instance) (id (gensym*)) 
                         (name transport-to-cs) (params ?wp ?ring-output ?cap-station ?cap-place ?cap-step)))
)

(defrule pddl-action-from-step-cap-to-delivery
    (pddl-fluent (name next-step) (params ?wp ?cap-step deliver))
    (test (str-index cap ?cap-step))
    (pddl-fluent (name step-place) (params ?cap-step ?cap-input))
    (pddl-fluent (name in) (params ?cap-station ?cap-input))
    (pddl-fluent (name out) (params ?cap-station ?cap-output))
    (confval (path "/pddl/problem_instance") (value ?instance-str))
    =>
    (bind ?instance (sym-cat ?instance-str))
    (assert (pddl-action (instance ?instance) (id (gensym*))
                         (name cs-mount-cap) (params ?wp ?cap-station ?cap-input ?cap-output ?cap-step deliver)))
    (assert (pddl-action (instance ?instance) (id (gensym*))
                         (name transport) (params ?wp ?cap-output ds-input deliver)))
    (assert (pddl-action (instance ?instance) (id (gensym*))
                         (name finalize) (params ?wp ds ds-input deliver)))
)

(defrule pddl-action-create-pay-with-carrier
    
)