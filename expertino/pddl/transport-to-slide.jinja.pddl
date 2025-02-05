   ;; Sub-action example for the transport-to-slide planning action.
   (:durative-action transport-to-slide-step-1-drive-to
     :parameters (?wp - workpiece ?from - place ?to - slide)
     :duration (= ?duration 5)
     :condition (and
       (at start (= (order) 0))
       (at start (at ?wp ?from))
       (at start (step ?wp dispose))                                            
       (at start (step-place dispose ?to)) 
       (at start (usable ?wp))
     )
     :effect (and
       (at start (not (usable ?wp)))
       (at end (increase (order) 1))
     )
   )

   (:durative-action transport-to-slide-step-2-pick-up
     :parameters (?wp - workpiece ?from - place ?to - place)
     :duration (= ?duration 5)
     :condition (and
       (at start (at ?wp ?from))
       (at start (= (order) 1))
     )
     :effect (and
       (at end (free ?from))
       (at end (increase (order) 1))
       (at end (not (at ?wp ?from)))
     )
   )

   (:durative-action transport-to-slide-step-4-place-down
     :parameters (?wp - workpiece ?from - place ?to - place)
     :duration (= ?duration 5)
     :condition (and
       (at start (= (order) 2))
       (at start (step ?wp dispose))                                            
       (at start (step-place dispose ?to)) 
     )
     :effect (and
       (at end (at ?wp ?to))
       (at end (usable ?wp))
       (at end (increase (order) 1))
     )
   )
