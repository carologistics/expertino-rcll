;(deffacts ring-specs
;  (ring-spec (color RING_BLUE) (cost 2))
;  (ring-spec (color RING_YELLOW) (cost 0))
;  (ring-spec (color RING_GREEN) (cost 1))
;  (ring-spec (color RING_ORANGE) (cost 1))
;)

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


;(deffacts new-order-O5
;  (order 
;    (id 5) 
;    (name O5) 
;    (workpiece nil) 
;    (complexity C3) 
;    (base-color BASE_SILVER) 
;    (ring-colors (create$ RING_GREEN RING_YELLOW RING_ORANGE))
;    (cap-color CAP_BLACK) 
;    (quantity-requested 1) 
;    (quantity-delivered 0) 
;    (quantity-delivered-other 0) 
;    (delivery-begin 383) 
;    (delivery-end 513) 
;    (competitive TRUE)))

;(deffacts new-order-O4
;  (order 
;    (id 4) 
;    (name O4) 
;    (workpiece nil) 
;    (complexity C3) 
;    (base-color BASE_RED) 
;    (ring-colors (create$ RING_YELLOW RING_GREEN RING_BLUE))
;    (cap-color CAP_GREY) 
;    (quantity-requested 1) 
;    (quantity-delivered 0) 
;    (quantity-delivered-other 0) 
;    (delivery-begin 295) 
;    (delivery-end 496) 
;    (competitive FALSE)))
  
;(deffacts new-order-O3
;  (order 
;    (id 3) 
;    (name O3) 
;    (workpiece nil) 
;    (complexity C3) 
;    (base-color BASE_RED) 
;    (ring-colors (create$ RING_BLUE RING_GREEN RING_BLUE))
;    (cap-color CAP_GREY) 
;    (quantity-requested 1) 
;    (quantity-delivered 0) 
;    (quantity-delivered-other 0) 
;    (delivery-begin 269) 
;    (delivery-end 309) 
;    (competitive FALSE)))
  

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
  (replan-required)  
  (insert-order (T-last-end ?last-end) (T-order-window ?window))
  ?order <- (order (id ?order-id) (name ?name))
  (order-scheduled (id ?order-id))
  ?o-f <- (order (name ?name) (workpiece nil) (base-color ?base-col) (ring-colors $?ring-cols) (cap-color ?cap-col)  (quantity-requested ?qty-requested)
            (quantity-delivered ?qty-delivered) (quantity-delivered-other ?qty-delivered-other) (delivery-begin ?delivery-begin) (delivery-end ?delivery-end) (competitive ?competitive) )
  (confval (path "/pddl/problem_instance") (value ?instance-str))
  =>
  (bind ?rcll (sym-cat ?instance-str))
  (bind ?wp (sym-cat (lowcase ?name) "-" (gensym*)))
  (assert (pending-pddl-object (instance ?rcll) (name ?wp) (type product)))
  (assert (pending-pddl-fluent (instance ?rcll) (name spawnable) (params ?wp)))
  (bind ?curr-step (wp-part-to-pddl ?base-col))
  (assert (pending-pddl-fluent (instance ?rcll) (name step) (params ?wp ?curr-step)))
  (bind ?ring-steps ?ring-cols)
  (bind ?ring-num 1)
  (while (neq ?ring-steps (create$))
    (bind ?next-step  (wp-part-to-pddl (nth$ 1 ?ring-steps) ?ring-num))
    (bind ?ring-steps  (rest$ ?ring-steps))
    (assert (pending-pddl-fluent (instance ?rcll) (name next-step) (params ?wp ?curr-step ?next-step)))
    (bind ?curr-step ?next-step)
    (bind ?ring-num (+ ?ring-num 1))
  )
  (assert (pending-pddl-fluent (instance ?rcll) (name next-step) (params ?wp ?curr-step ?next-step)))
  (assert (pending-pddl-fluent (instance ?rcll)  (name next-step) (params ?wp ?next-step deliver)))
  (assert (pending-pddl-fluent (instance ?rcll) (name next-step) (params ?wp deliver done)))
  (assert (pddl-goal-fluent (instance ?rcll) (name step) (params ?wp done)))
  (assert (pddl-clear-goals (instance ?rcll)))
)

(defrule set-goal-for-orders
  (startup-completed)
  (pddl-goal-fluent (instance ?rcll) (name step) (params ?wp1 $?))
  (pddl-goal-fluent (instance ?rcll) (name step) (params ?wp2&:(neq ?wp1 ?wp2) $?))
  (pddl-goal-fluent (instance ?rcll) (name step) (params ?wp3&:(neq ?wp1 ?wp3) &:(neq ?wp2 ?wp3) $?))
  ?clear-f <- (pddl-clear-goals (instance ?rcll) (state DONE))
  =>
  (printout t "Setting goals for instance: " crlf)
  (assert (pddl-set-goals (instance ?rcll))) 
  (retract ?clear-f)
  (printout t "Goals set successfully!" crlf)
)


;(defrule update-hardcoded-plan
;  (replan-required)  
;  (insert-order (T-last-end ?last-end) (T-order-window ?window))  
;   ?order <- (order (id ?order-id) (name ?name))
;  (order-scheduled (id ?order-id))
;  =>
;  (printout t "Replanning required. Updating the plan ..." crlf)
;  (assert (plan-step (id 6) (task dispense) (start-time (+ ?last-end 5)) (duration 5))) 
;  (assert (plan-step (id 7) (task transport) (start-time (+ ?last-end 10)) (duration 20)))  
;  (assert (plan-step (id 8) (task mount) (start-time (+ ?last-end 30)) (duration 10)))
;  (assert (plan-step (id 9) (task transport) (start-time (+ ?last-end 40)) (duration 20)))
;  (assert (plan-step (id 10) (task finalize) (start-time (+ ?last-end 60)) (duration 5)))
;  (printout t "Plan successfully updated." crlf)
;)

(defrule goal-updated-start-plan
  (startup-completed)
  ?set-f <- (pddl-set-goals (instance ?rcll) (state DONE))
  (pddl-manager (node ?node))
  (pddl-instance (name ?rcll)  (busy-with FALSE) (state LOADED))  
  (planning-filter (action-names $?an))
  (expertino-msgs-plan-temporal-client (server ?server&:(eq ?server (str-cat ?node "/temp_plan"))))
  (not (planned-for-main))
  =>
  (printout green "Start planning for instance: " ?rcll crlf)  
  (bind ?goal (expertino-msgs-plan-temporal-goal-create))
  (assert (pddl-planner-call (context test-plan) (goal ?goal)))
  (printout t "Planner goal created: " ?goal crlf)  
  (expertino-msgs-plan-temporal-goal-set-field ?goal "pddl_instance" ?rcll)  
  (expertino-msgs-plan-temporal-goal-set-field ?goal "action_names" ?an)
  (expertino-msgs-plan-temporal-send-goal ?goal ?server)
  (assert (planned-for-main))
  (printout t "Planning request sent to server: " ?server crlf)
)




