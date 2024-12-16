(deffacts test-order
  (order (id 1) (name O1) (complexity C3) (base-color BASE_RED) (ring-colors (create$)) (cap-color CAP_GREY))
  (order (id 2) (name O2) (complexity C3) (base-color BASE_BLACK) (ring-colors (create$ RING_BLUE RING_YELLOW)) (cap-color CAP_BLACK))
  (ring-spec (color RING_BLUE) (cost 2))
  (ring-spec (color RING_YELLOW) (cost 0))
  (ring-spec (color RING_GREEN) (cost 1))
  (ring-spec (color RING_ORANGE) (cost 1))
)

(deffunction wp-part-to-pddl (?refbox-name $?number)
  (printout blue ?refbox-name crlf)
  (bind ?converted-name (str-replace (lowcase ?refbox-name) "_" "-"))
  (if (neq ?number (create$)) then
    (return (sym-cat ?converted-name (nth$ 1 ?number)))
  else
    (return ?converted-name)
  )
)

(defrule add-order-to-problem
  (startup-completed)
  ?o-f <- (order (name ?name) (workpiece nil) (base-color ?base-col) (ring-colors $?ring-cols) (cap-color ?cap-col))
  (confval (path "/pddl/problem_instance") (value ?instance-str))
  =>
  (bind ?instance (sym-cat ?instance-str))
  (bind ?wp (sym-cat (lowcase ?name) "-" (gensym*)))
  (assert (pending-pddl-object (instance ?instance) (name ?wp) (type product)))
  (assert (pending-pddl-fluent (instance ?instance) (name spawnable) (params ?wp)))
  (bind ?curr-step (wp-part-to-pddl ?base-col))
  (assert (pending-pddl-fluent (instance ?instance) (name step) (params ?wp ?curr-step)))
  (bind ?ring-steps ?ring-cols)
  (bind ?ring-num 1)
  (while (neq ?ring-steps (create$))
    (bind ?next-step  (wp-part-to-pddl (nth$ 1 ?ring-steps) ?ring-num))
    (bind ?ring-steps  (rest$ ?ring-steps))
    (assert (pending-pddl-fluent (instance ?instance) (name next-step) (params ?wp ?curr-step ?next-step)))
    (bind ?curr-step ?next-step)
    (bind ?ring-num (+ ?ring-num 1))
  )
  (bind ?next-step (wp-part-to-pddl ?cap-col))
  (assert (pending-pddl-fluent (instance ?instance) (name next-step) (params ?wp ?curr-step ?next-step)))
  (assert (pending-pddl-fluent (instance ?instance) (name next-step) (params ?wp ?next-step deliver)))
  (assert (pending-pddl-fluent (instance ?instance) (name next-step) (params ?wp deliver done)))
  ; set the wp as a goal
  (assert (pddl-goal-fluent (instance ?instance) (name step) (params ?wp done)))
  ; also, clear all old goals
  (assert (pddl-clear-goals (instance ?instance)))
)

(defrule set-goal-for-orders
  (startup-completed)
  (pddl-goal-fluent (instance ?instance) (name step) (params ?wp1 $?))
  (pddl-goal-fluent (instance ?instance) (name step) (params ?wp2&:(neq ?wp1 ?wp2) $?))
  ?clear-f <- (pddl-clear-goals (instance ?instance) (state DONE))
  =>
  ; notify to add the goal to the domain
  (assert (pddl-set-goals (instance ?instance)))
  (retract ?clear-f)
)

(defrule goal-updated-start-plan
  (startup-completed)
  ?set-f <- (pddl-set-goals (instance ?instance) (state DONE))
  (pddl-manager (node ?node))
  (pddl-instance (name ?instance))
  (expertino-msgs-plan-temporal-client (server ?server&:(eq ?server (str-cat ?node "/temp_plan"))))
  (not (planned-for-main))
  =>
  (printout green "Start planning" crlf)
  (bind ?goal (expertino-msgs-plan-temporal-goal-create))
  (assert (pddl-planner-call (context test-plan) (goal ?goal)))
  (expertino-msgs-plan-temporal-goal-set-field ?goal "pddl_instance" ?instance)
  (expertino-msgs-plan-temporal-send-goal ?goal ?server)
  (assert (planned-for-main))
)

