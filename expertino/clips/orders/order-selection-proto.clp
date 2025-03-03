;; Define global constant for the simulation time limit.
(defglobal 
?*MAX-TIMESTEPS* = 20
?*C0_COMPLETION_TIME* = 4
?*C1_COMPLETION_TIME* = 6
?*C2_COMPLETION_TIME* = 8
?*C3_COMPLETION_TIME* = 10
)

;; Templates for incoming orders, selected orders, and the current simulation time.
(deftemplate Order
   (slot id)
   (slot arrival-time)
   (slot process-time)
   (slot window-start)
   (slot window-end))

(deftemplate order
  (slot id (type INTEGER))
  (slot name (type SYMBOL))
  (slot workpiece (type SYMBOL))
  (slot complexity (type SYMBOL))

  (slot base-color (type SYMBOL))
  (multislot ring-colors (type SYMBOL))
  (slot cap-color (type SYMBOL))

  (slot quantity-requested (type INTEGER))
  (slot quantity-delivered (type INTEGER))
  (slot quantity-delivered-other (type INTEGER))

  (slot delivery-begin (type INTEGER))
  (slot delivery-end (type INTEGER))
  (slot competitive (type SYMBOL))
)

(deftemplate SelectedOrder
   (slot id)
   (slot start-time)
   (slot finish-time)
   (slot punctuality))  ;; "on-time" or "tardy"

(deftemplate CurrentTime
   (slot time))

;; Initial facts: the simulation clock and the four orders.
(deffacts initial-facts
   (CurrentTime (time 0))
   (order (id 1) (arrival-time 2)  (process-time 4) (window-start 8)  (window-end 12))
   (order (id 2) (arrival-time 1)  (process-time 4) (window-start 17) (window-end 20))
   (order (id 3) (arrival-time 4)  (process-time 9) (window-start 8)  (window-end 12))
   (order (id 4) (arrival-time 12) (process-time 6) (window-start 15) (window-end 18))
)

;; Rule: If an order is available (has arrived) and processing it now would finish before the time limit,
;; evaluate the order and, if its finish time is acceptable, select it.
(defrule select-order
   (declare (salience 10))
   ?ctFact <- (CurrentTime (time ?ct))
   ?order <- (Order (id ?id) (arrival-time ?at) (process-time ?pt)
                    (window-start ?ws) (window-end ?we))
   (test (<= ?at ?ct)) ;; order must have arrived
   (test (<= (+ ?ct ?pt) ?*MAX-TIMESTEPS*)) ;; can complete before the global limit
   =>
   ;; In this simulation, since the order has arrived,
   ;; we use the current time as the starting time.
   (bind ?earliest ?ct)
   (bind ?finish (+ ?earliest ?pt))
   (bind ?window-length (- ?we ?ws))
   (bind ?max-finish (+ ?we ?window-length))
   (if (<= ?finish ?ws) then
         (printout t "Order " ?id " not selected: finish time " ?finish 
                   " is before the allowed window start " ?ws "." crlf)
      else
         (if (<= ?finish ?we) then
              (assert (SelectedOrder (id ?id)
                                      (start-time ?earliest)
                                      (finish-time ?finish)
                                      (punctuality "on-time")))
           else
              (if (<= ?finish ?max-finish) then
                   (assert (SelectedOrder (id ?id)
                                           (start-time ?earliest)
                                           (finish-time ?finish)
                                           (punctuality "tardy")))
                else
                   (printout t "Order " ?id " not selected: finish time " ?finish 
                             " exceeds maximum allowed " ?max-finish "." crlf)
              )
         )
   )
   (retract ?order)
   (modify ?ctFact (time (+ ?ct 1)))
   (printout t "Time advanced to " (+ ?ct 1) "." crlf)
)

;; Rule: Process a selected order (simulate execution by printing messages)
(defrule process-selected-order
   (declare (salience 5))
   ?so <- (SelectedOrder (id ?id) (start-time ?st) (finish-time ?ft) (punctuality ?punct))
   =>
   (printout t "Processing Order " ?id " starting at time " ?st "..." crlf)
   (printout t "Order " ?id " completed at time " ?ft " (" ?punct ")." crlf)
   (retract ?so)
)

;; Rule: If no order has arrived by the current time, just advance the clock.
(defrule tick
   (declare (salience 0))
   ?ctFact <- (CurrentTime (time ?ct))
   (test (< ?ct ?*MAX-TIMESTEPS*))
   ;; Check that no order in the fact base has arrived at or before the current time.
   (not (Order (arrival-time ?at&:(<= ?at ?ct))))
   =>
   (modify ?ctFact (time (+ ?ct 1)))
   (printout t "No orders available at time " ?ct ". Time advanced to " (+ ?ct 1) "." crlf)
)

;; Rule: End simulation when the time limit is reached.
(defrule finish-simulation
   (declare (salience -10))
   ?ctFact <- (CurrentTime (time ?ct))
   (test (>= ?ct ?*MAX-TIMESTEPS*))
   =>
   (printout t crlf "All order processes completed or time limit reached." crlf)
)
