   ;; Sub-action example for the transport planning action.
   (:durative-action transport-step-1-drive-to
     :parameters (?wp - workpiece ?from - place ?to - place ?r - task)
     :duration (= ?duration 5)
     :condition (and
       (at start (= (order) 0))
       (at start (at ?wp ?from))
       (at start (step ?wp ?r))
       (at start (step-place ?r ?to))
       (at start (free ?to))
       (at start (usable ?wp))
       (over all (free ?to))
     )
     :effect (and
       (at start (not (usable ?wp)))
       (at end (increase (order) 1))
     )
   )

   (:durative-action transport-step-2-pick-up
     :parameters (?wp - workpiece ?from - place ?to - place ?r - task)
     :duration (= ?duration 5)
     :condition (and
       (at start (at ?wp ?from))
       (at start (= (order) 1))
     )
     :effect (and
       (at end (free ?from))
       (at end (increase (order) 1))
     )
   )

   (:durative-action transport-step-4-place-down
     :parameters (?wp - workpiece ?from - place ?to - place ?r - task)
     :duration (= ?duration 5)
     :condition (and
       (at start (= (order) 2))
       (at start (step ?wp ?r))
       (at start (step-place ?r ?to))
       (at start (free ?to))
       (over all (free ?to))
     )
     :effect (and
       (at end (at ?wp ?to))
       (at end (not (free ?to)))
       (at end (usable ?wp))
       (at end (increase (order) 1))
     )
   )
