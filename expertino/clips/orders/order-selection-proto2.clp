(defglobal ?*MAX-TIMESTEPS* = 20)
(defglobal ?*CURRENT-TIME* = 0)

(deftemplate Order
   (slot id)
   (slot arrival-time)
   (slot process-time)
   (slot window-start)
   (slot window-end))

(deftemplate SelectedOrder
   (slot id)
   (slot start-time)
   (slot finish-time)
   (slot punctuality))

(deftemplate CurrentTime
   (slot value))

(deffacts initial-facts
   (CurrentTime (value 0)))

(deffacts orders
   (Order (id 1) (arrival-time 2) (process-time 4) (window-start 8) (window-end 12))
   (Order (id 2) (arrival-time 5) (process-time 4) (window-start 7) (window-end 11))
   (Order (id 3) (arrival-time 4) (process-time 9) (window-start 8) (window-end 12))
   (Order (id 4) (arrival-time 10) (process-time 6) (window-start 15) (window-end 18)))

(defrule evaluate-order
   ?ct <- (CurrentTime (value ?current-time))
   ?order <- (Order (id ?id) (arrival-time ?arrival-time) (process-time ?process-time)
                    (window-start ?window-start) (window-end ?window-end))
   (test (<= ?current-time ?*MAX-TIMESTEPS*))
   =>
   (bind ?window-length (- ?window-end ?window-start))
   (bind ?max-finish (+ ?window-end ?window-length))
   (bind ?earliest-start (max ?current-time ?arrival-time))
   (bind ?finish-time (+ ?earliest-start ?process-time))
   
   (if (<= ?finish-time ?*MAX-TIMESTEPS*)
      then
      (if (<= ?finish-time ?window-start)
         then (bind ?punctuality "early")
         else (if (<= ?finish-time ?window-end)
                 then (bind ?punctuality "on-time")
                 else (if (<= ?finish-time ?max-finish)
                         then (bind ?punctuality "tardy")
                         else (return))))
      
      (assert (SelectedOrder (id ?id) (start-time ?earliest-start) 
                             (finish-time ?finish-time) (punctuality ?punctuality)))
      (retract ?order)
      (modify ?ct (value (+ ?current-time 1)))
      (printout t "Processing Order " ?id " starting at time " ?earliest-start "..." crlf)
      (printout t "Order " ?id " completed at time " ?finish-time "." crlf)
    else
      (modify ?ct (value (+ ?current-time 1)))))

(defrule finish-simulation
   (declare (salience -10))
   ?ct <- (CurrentTime (value ?time))
   (test (>= ?time ?*MAX-TIMESTEPS*))
   =>
   (printout t crlf "All order processes completed or time limit reached." crlf crlf)
   (printout t "Final Selected and Processed Orders:" crlf)
   (do-for-all-facts ((?so SelectedOrder)) TRUE
      (printout t "Order ID: " ?so:id ", Start Time: " ?so:start-time 
                ", Finish Time: " ?so:finish-time ", " ?so:punctuality crlf)))

(defrule no-orders-processed
   (declare (salience -20))
   (not (SelectedOrder))
   =>
   (printout t "No orders were selected and processed." crlf))
