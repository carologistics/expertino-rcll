; Define an Order with basic properties:
(deftemplate Order
  (slot id)
  (slot start-time)
  (slot process-time)
  (slot window-start)   ; earliest allowed delivery time
  (slot window-end))    ; ideal (deadline) delivery time

; Define a SelectedOrder that records the finish time and if it is on-time or tardy.
(deftemplate SelectedOrder
  (slot id)
  (slot finish-time)
  (slot punctuality))   ; either "on-time" or "tardy"

; Assert three dummy orders.
(defrule init-orders
  =>
  (assert (Order (id 1) (start-time 2) (process-time 4) (window-start 8) (window-end 12)))
  (assert (Order (id 2) (start-time 5) (process-time 4) (window-start 8) (window-end 12)))
  (assert (Order (id 3) (start-time 4) (process-time 9) (window-start 8) (window-end 12)))
  (printout t "Initial orders asserted." crlf))

; This rule evaluates each order based on its computed finish time.
(defrule select-order
  ?o <- (Order (id ?id)
               (start-time ?start)
               (process-time ?pt)
               (window-start ?ws)
               (window-end ?we))
  =>
  ; Compute the finish time (when processing would complete)
  (bind ?finish (+ ?start ?pt))
  ; Calculate the allowed tardiness: here we allow extra time equal to the ideal window length.
  (bind ?delivery-window-length (- ?we ?ws))
  (bind ?max-finish (+ ?we ?delivery-window-length))
  
  (if (< ?finish ?ws) then
      (printout t "Order " ?id " not selected: completion too early (" ?finish " is before window-start " ?ws ")." crlf)
      (retract ?o)
    else
      (if (<= ?finish ?we) then
          (assert (SelectedOrder (id ?id) (finish-time ?finish) (punctuality on-time)))
          (printout t "Order " ?id " selected as on-time (" ?finish " is within the window [" ?ws ", " ?we "])." crlf)
        else
          (if (<= ?finish ?max-finish) then
              (assert (SelectedOrder (id ?id) (finish-time ?finish) (punctuality tardy)))
              (printout t "Order " ?id " selected as tardy (" ?finish " is after the window, but before the max allowed " ?max-finish ")." crlf)
            else
              (printout t "Order " ?id " not selected: completion too late (" ?finish " is after the allowed maximum " ?max-finish ")." crlf)
              (retract ?o)
          )
      )
  )
)

; Optional rule to display all selected orders.
(defrule show-selected-orders
  ?so <- (SelectedOrder (id ?id) (finish-time ?ft) (punctuality ?punct))
  =>
  (printout t "Final Selected Order: ID " ?id ", finish-time = " ?ft ", " ?punct crlf)
)
