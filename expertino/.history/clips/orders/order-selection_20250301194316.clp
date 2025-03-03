(deftemplate T-last-end
  (slot value))

(deftemplate T-order-window
  (slot value))

(deftemplate plan-step
  (slot id)
  (slot task)
  (slot start-time)
  (slot duration))

(deftemplate insert-order
  (slot T-last-end)
  (slot T-order-window))


(deftemplate order-scheduled
  (slot id))


(deffacts initial-hardcoded-plan
  (plan-step (id 1) (task dispense) (start-time 0) (duration 5))
  (plan-step (id 2) (task transport) (start-time 5) (duration 20))
  (plan-step (id 3) (task mount) (start-time 25) (duration 10))
  (plan-step (id 4) (task transport) (start-time 35) (duration 20))
  (plan-step (id 5) (task finalize) (start-time 195) (duration 5)))


(defrule compute-last-end-time
  (not (T-last-end)) 
  =>
  (bind ?max-end 0)
  (do-for-all-facts ((?p plan-step)) TRUE
    (bind ?step-end (+ (fact-slot-value ?p start-time) (fact-slot-value ?p duration)))
    (if (> ?step-end ?max-end) then
        (bind ?max-end ?step-end)))
  (assert (T-last-end (value ?max-end))) 
  (printout t "Computed last end time: " ?max-end crlf))


(defrule compute-order-window
  (order (id ?order-id) (delivery-begin ?begin) (delivery-end ?end))
  =>
  (bind ?T_order_window (- ?end ?begin))
  (assert (T-order-window (value ?T_order_window)))
  (printout t "Computed scheduling window for order " ?order-id ": " ?T_order_window crlf))


(defrule check-order-scheduling
  (T-last-end (value ?T_last_end))
  (T-order-window (value ?T_order_window))
  (order (id ?order-id) (delivery-begin ?delivery-begin) (delivery-end ?delivery-end))
  (not (order-scheduled (id ?order-id)))  
  =>
  (printout t "Checking scheduling for order " ?order-id ": Last End Time = " ?T_last_end ", Scheduling Window = " ?T_order_window ", Delivery Begin = " ?delivery-begin ", Delivery End = " ?delivery-end crlf)
  (bind ?available-gap (- ?delivery-end ?delivery-begin))
  (printout t "Available gap for order " ?order-id ": " ?available-gap crlf)
  (if (< ?T_last_end ?available-gap)
    then
      (assert (insert-order (T-last-end ?T_last_end) (T-order-window ?T_order_window)))
      (assert (replan-required))
      (assert (order-scheduled (id ?order-id)))   
      (printout t "Order " ?order-id " can be scheduled! Requesting planner update..." crlf)
    else
      (printout t "Order " ?order-id " cannot be scheduled yet. Waiting for another order..." crlf)
))


(deffunction wp-part-to-pddl (?refbox-name $?number)
  (bind ?converted-name (str-replace (lowcase ?refbox-name) "_" "-"))
  (if (neq ?number (create$)) then
    (return (sym-cat ?converted-name (nth$ 1 ?number)))
  else
    (return ?converted-name)
  )
)


(defrule add-order-to-problem
  (startup-completed)
  ;(replan-required)  
  ;(insert-order (T-last-end ?last-end) (T-order-window ?window))
  ;(order-scheduled (id ?order-id))
  ?o-f <- (order (id ?order-id) (name ?name) (workpiece nil)  (base-color ?base-col) (ring-colors $?ring-cols) (cap-color ?cap-col)  (quantity-requested ?qty-requested)
            (quantity-delivered ?qty-delivered) (quantity-delivered-other ?qty-delivered-other) (delivery-begin ?delivery-begin) (delivery-end ?delivery-end) (competitive ?competitive) (state OPEN))
  ;(not (added-one-order)) ;remove this eventually
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
  (modify ?o-f (state ACTIVE))
  ; set the wp as a goal
  (assert (pddl-goal-fluent (instance ?instance) (name step) (params ?wp done)))
  ; also, clear all old goals
  (assert (pddl-clear-goals (instance ?instance)))
  ;(assert (added-one-order))
)

(defrule set-goal-for-orders
  (startup-completed)
  (pddl-goal-fluent (instance ?instance) (name step) (params ?wp1 $?))
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
  (pddl-instance (name ?instance) (busy-with FALSE) (state LOADED))
  (planning-filter (action-names $?an))
  (expertino-msgs-plan-temporal-client (server ?server&:(eq ?server (str-cat ?node "/temp_plan"))))
  (not (planned-for-main))
  =>
  (printout green "Start planning" crlf)
  (bind ?goal (expertino-msgs-plan-temporal-goal-create))
  (assert (pddl-planner-call (context test-plan) (goal ?goal)))
  (expertino-msgs-plan-temporal-goal-set-field ?goal "pddl_instance" ?instance)
  (expertino-msgs-plan-temporal-goal-set-field ?goal "action_names" ?an)
  (expertino-msgs-plan-temporal-send-goal ?goal ?server)
  (assert (planned-for-main))
)

;(defrule action-apply-effect-test-main-action
;" Showcase how to apply an action effect 'directly' (as in, the actual action
;  of the plan is used).
;  It applies all at-start and at-end effects of it are applied.
;  Hence, WP o2-gen1 ends up being at a BS side in the worldmodel once this is
;  processed.
;"
;  (pddl-action (instance rcll) (id ?action-id) (name bs-dispense) (params ?wp bs ?bs-side base-black ring-blue1))
;  =>
;  (assert (pddl-action-apply-effect (action ?action-id) (effect-type ALL)))
;)
;
;(defrule action-apply-effect-test-sub-action
;" Continuation of the example for applying effects, showcasing how to handle
;  sub-actions and partial effect application.
;  Once the action-apply-effect-test-main-action activation caused the
;  bs-dispense action effects to be applied, this rule takes the subsequent
;  transport action and applies some partial effects.
;  In particular, it applies the at-start effects of the 'transport-step-1-drive-to'
;  action by first creating a suitable grounded pddl-action for it and then
;  requesting the application of the effects.
;"
;  (pddl-action (instance rcll) (id ?dispense-id) (name bs-dispense))
;  ?apply-effect <- (pddl-action-apply-effect (action ?dispense-id) (state DONE))
;  (pddl-action (instance rcll) (id ?transport) (name transport) (params ?wp ?bs-side rs2-input ring-blue1))
;  =>
;  (retract ?apply-effect)
;  (bind ?id (gensym*))
;  (assert (pddl-action (instance rcll) (id ?id) (name transport-step-1-drive-to) (params ?wp ?bs-side rs2-input ring-blue1)))
;  (assert (pddl-action-apply-effect (action ?id) (effect-type START)))
;)