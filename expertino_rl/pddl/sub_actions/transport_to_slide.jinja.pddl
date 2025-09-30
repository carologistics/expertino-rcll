   ;; Sub-action example for the transport-to-slide action.
   (:durative-action transport-to-slide-step-1-drive-to
     :parameters (?token - payment ?from - bs-place ?to - slide ?rs - ring-station)  
     :duration (= ?duration 5)
     :condition (and
       (at start (<= (pay-count ?rs) 2)) 
       (at start (at ?token ?from))
       (at start (token-step ?token ?from dispose)) 
       (at start (token-usable ?token ?from))
     )
     :effect (and
       (at start (not (token-usable ?token ?from)))
     )
   )

   (:durative-action transport-to-slide-step-2-pick-up
     :parameters (?token - payment ?from - bs-place ?to - slide ?rs - ring-station)  
     :duration (= ?duration 5)
     :condition (and
       (at start (<= (pay-count ?rs) 2)) 
       (at start (at ?token ?from))
     )
     :effect (and
       (at end (free ?from))
       (at end (not (at ?token ?from)))
     )
   )

   (:durative-action transport-to-slide-step-4-place-down
     :parameters (?token - payment ?from - bs-place ?to - slide ?rs - ring-station)  
     :duration (= ?duration 5)
     :condition (and
       (at start (<= (pay-count ?rs) 2)) 
       (at start (token-step ?token ?from dispose)) 
       (at start (step-place dispose ?to)) 
       (at start (rs-slide ?rs ?to)) 
     )
     :effect (and
       (at end (increase (pay-count ?rs) 1)) 
     )
   )
