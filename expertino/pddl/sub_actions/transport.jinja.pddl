   ;; Sub-action example for the transport planning action.
   (:durative-action transport-step-1-drive-to
     :parameters (?prod - product ?from - place ?to - place ?step - step-name)
     :duration (= ?duration 5)
     :condition (and
       (at start (at ?prod ?from))
       (at start (step ?prod ?step))
       (at start (step-place ?step ?to))
       (at start (free ?to))
       (at start (usable ?prod))
       (over all (free ?to))
     )
     :effect (and
       (at start (not (usable ?prod)))
     )
   )

   (:durative-action transport-step-2-pick-up
     :parameters (?prod - product ?from - place ?to - place ?step - step-name)
     :duration (= ?duration 5)
     :condition (and
       (at start (at ?prod ?from))
     )
     :effect (and
       (at end (free ?from))
       (at end (not (at ?prod ?from)))
     )
   )

   (:durative-action transport-step-4-place-down
     :parameters (?prod - product ?from - place ?to - place ?step - step-name)
     :duration (= ?duration 5)
     :condition (and
       (at start (step ?prod ?step))
       (at start (step-place ?step ?to))
       (at start (free ?to))
       (over all (free ?to))
     )
     :effect (and
       (at end (at ?prod ?to))
       (at end (not (free ?to)))
       (at end (usable ?prod))
     )
   )
