(deffunction wp-part-to-pddl (?refbox-name $?number)
  (bind ?converted-name (str-replace (lowcase ?refbox-name) "_" "-"))
  (if (neq ?number (create$)) then
    (return (sym-cat ?converted-name (nth$ 1 ?number)))
  else
    (return ?converted-name)
  )
)

(defrule add-ring-specs-to-problem
  (startup-completed)
  (not (added-ring-specs))
  (confval (path "/pddl/problem_instance") (value ?instance-str))
  (ring-spec (color RING_GREEN))
  (ring-spec (color RING_YELLOW))
  (ring-spec (color RING_BLUE))
  (ring-spec (color RING_ORANGE))
  =>
  (bind ?instance (sym-cat ?instance-str))
  (delayed-do-for-all-facts ((?ring-spec ring-spec)) TRUE
    (bind ?value (float ?ring-spec:cost))
    (foreach ?i (create$ 1 2 3)
      (assert (pending-pddl-numeric-fluent (instance ?instance) (name price)
                 (params (ring-color-to-pddl ?ring-spec:color ?i))
                 (value ?value)))
    )
  )
  (assert (added-ring-specs))
)

(defrule add-order-to-problem
  (startup-completed)
  (added-ring-specs)
  ?o-f <- (order (id ?order-id) (workpiece nil) (base-color ?base-col) (ring-colors $?ring-cols) (cap-color ?cap-col))
  (production-strategy-order-filter (name selected-orders) (orders $?orders&:(member$ ?order-id ?orders))) ;revise this later
  (not (workpiece-for-order (order ?order-id)))
  (confval (path "/pddl/problem_instance") (value ?instance-str))
  =>
  (bind ?instance (sym-cat ?instance-str))
  (bind ?wp (sym-cat (lowcase ?order-id) "-" (gensym*)))
  (assert (workpiece-for-order (wp ?wp) (order ?order-id)))
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
  (assert (pddl-clear-goals (instance ?instance) (goal ?*GOAL-INSTANCE-BASE*)))
  ; set the wp as a goal
  (assert (pddl-goal-fluent (instance ?instance) (goal ?*GOAL-INSTANCE-BASE*) (name step) (params ?wp done)))
)

(defrule remove-achieved-goal
  ?goal-f <- (pddl-goal-fluent (instance ?instance) (name ?f-name) (params $?f-params))
  (pddl-fluent (instance ?instance) (name ?f-name) (params $?f-params))
  =>
  (retract ?goal-f)
)

(defrule set-goal-for-orders
  (startup-completed)
  (pddl-goal-fluent (instance ?instance) (goal ?goal&:(eq ?goal ?*GOAL-INSTANCE-BASE*)) (name step) (params ?wp1 $?))
  ?clear-f <- (pddl-clear-goals (instance ?instance) (state DONE) (goal ?goal&:(eq ?goal ?*GOAL-INSTANCE-BASE*)))
  =>
  ; notify to add the goal to the domain
  (assert (pddl-set-goals (instance ?instance) (goal ?*GOAL-INSTANCE-BASE*)))
  ;freeze the current agenda execution
  (if (any-factp ((?agenda agenda) (?plan pddl-plan)) (and (eq ?agenda:plan ?plan:id) (eq ?plan:instance ?instance)))
   then
    (assert (freeze-agenda (instance ?instance)))
  )
  (retract ?clear-f)
)

(defrule goal-updated-start-planning-for-orders
  (startup-completed)
  ?set-f <- (pddl-set-goals (instance ?instance) (state DONE) (goal ?goal&:(eq ?goal ?*GOAL-INSTANCE-BASE*)))
  (pddl-manager (node ?node))
  (pddl-instance (name ?instance) (busy-with FALSE) (state LOADED))
  (expertino-msgs-plan-temporal-client (server ?server&:(eq ?server (str-cat ?node "/temp_plan"))))
  ;(not (planned-for-main))
  (not (and 
        (agenda (plan ?plan-id) (state ACTIVE))
        (pddl-plan (id ?plan-id) (instance ?instance))
       )
  )
  (not (freeze-agenda (instance ?instance)))
  =>
  (printout green "Start planning" crlf)
  (bind ?goal (expertino-msgs-plan-temporal-goal-create))
  (assert (pddl-planner-call (context test-plan) (goal ?goal)))
  (expertino-msgs-plan-temporal-goal-set-field ?goal "pddl_instance" ?instance)
  (expertino-msgs-plan-temporal-goal-set-field ?goal "goal_instance" (str-cat ?*GOAL-INSTANCE-BASE*))
  (expertino-msgs-plan-temporal-send-goal ?goal ?server)
  ;(assert (planned-for-main))
  (retract ?set-f)
  ;clear all old goals in pddl_manager                                           
  (do-for-all-facts ((?goal-fluent pddl-goal-fluent))                            
    (and (eq ?goal-fluent:instance ?instance) (eq ?goal-fluent ?goal))          
    (retract ?goal-fluent)                                                       
  )                                                                              
  (do-for-all-facts ((?goal-fluent pddl-goal-numeric-fluent))                    
    (and (eq ?goal-fluent:instance ?instance) (eq ?goal-fluent ?goal))          
    (retract ?goal-fluent)                                                       
  )
)

(defrule remove-delivered-product-objects
  (confval (path "/pddl/problem_instance") (value ?instance-str))
  (pddl-fluent (instance ?instance&:(eq ?instance (sym-cat ?instance-str))) (name step) (params ?wp done))
  =>
  (assert (pending-pddl-object (instance ?instance) (name ?wp) (type product) (delete TRUE)))
)
