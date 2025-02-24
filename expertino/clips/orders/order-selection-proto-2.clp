;; Define global constants for the simulation time limit and complexity processing times.
(defglobal
   ?*MAX-TIMESTEPS* = 1200
   ?*PRODUCTION_STRAT_C0_COMPLETION_TIME* = 240
   ?*PRODUCTION_STRAT_C1_COMPLETION_TIME* = 360
   ?*PRODUCTION_STRAT_C2_COMPLETION_TIME* = 480
   ?*PRODUCTION_STRAT_C3_COMPLETION_TIME* = 600
)

(deftemplate SelectedOrder
   (slot id (type INTEGER))
   (slot start-time (type INTEGER))
   (slot finish-time (type INTEGER))
   (slot punctuality (type SYMBOL))
)

;; Rule: Select and process orders based on complexity and delivery window
(defrule select-order
   (declare (salience 10))
   ?ctFact <- (game-time ?gt)
   ?order <- (order (id ?id) (complexity ?c) (delivery-begin ?ws) (delivery-end ?we))
   =>
   (bind ?pt 0)
   ;; Determine processing time based on complexity
   (if (eq ?c C0)
      then (bind ?pt ?*PRODUCTION_STRAT_C0_COMPLETION_TIME*)
      else (if (eq ?c C1)
         then (bind ?pt ?*PRODUCTION_STRAT_C1_COMPLETION_TIME*)
         else (if (eq ?c C2)
            then (bind ?pt ?*PRODUCTION_STRAT_C2_COMPLETION_TIME*)
            else (if (eq ?c C3)
               then (bind ?pt ?*PRODUCTION_STRAT_C3_COMPLETION_TIME*)
               else (printout t "Unknown complexity " ?c crlf)
            )
         )
      )
   )
   ;; Check if processing can complete within time limit
   (if (<= (+ ?gt ?pt) ?*MAX-TIMESTEPS*)
      then
         (bind ?earliest ?gt)
         (bind ?earliest-finish (+ ?earliest ?pt))
         (bind ?window-length (- ?we ?ws))
         (bind ?max-finish (+ ?we ?window-length))
         (if (<= ?earliest-finish ?ws)
            then
               (printout t "Order " ?id " not selected: finish before window @ " ?finish crlf)
            else
               (if (<= ?earliest-finish ?we)
                  then
                     (assert (SelectedOrder (id ?id) (start-time ?earliest)
                                             (finish-time ?earliest-finish) 
                                             (punctuality nil)))
                  else
                     (if (<= ?earliest-finish ?max-finish)
                        then
                           (assert (SelectedOrder (id ?id) (start-time ?earliest)
                                                   (finish-time ?earliest-finish) 
                                                   (punctuality nil)))
                        else
                           (printout t "Order " ?id " rejected: exceeds max finish @ " ?max-finish crlf)
                     )
                (retract ?order)
               )
         )
      else
         (printout t "Order " ?id " rejected: exceeds global time limit" crlf)

   )
)