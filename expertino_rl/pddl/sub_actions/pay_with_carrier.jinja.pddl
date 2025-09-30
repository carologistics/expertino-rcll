   ;; Sub-action example for the pay-with-carrier action.
   (:durative-action pay-with-carrier-step-1-drive-to
     :parameters (?m - ring-station ?carrier - carrier ?from - place ?to - slide)
     :duration (= ?duration 5)
     :condition (and
       (at start (<= (pay-count ?m) 2)) 
       (at start (at ?carrier ?from))
       (at start (token-step ?carrier ?from dispose)) 
       (at start (token-usable ?carrier ?from))
     )
     :effect (and
       (at start (not (token-usable ?carrier ?from)))
     )
   )

   (:durative-action pay-with-carrier-step-2-pick-up
     :parameters (?m - ring-station ?carrier - carrier ?from - place ?to - slide)
     :duration (= ?duration 5)
     :condition (and
       (at start (<= (pay-count ?m) 2)) 
       (at start (at ?carrier ?from))
     )
     :effect (and
       (at end (free ?from))
       (at end (not (at ?carrier ?from)))
     )
   )

   (:durative-action pay-with-carrier-step-4-place-down
     :parameters (?m - ring-station ?carrier - carrier ?from - place ?to - slide)
     :duration (= ?duration 5)
     :condition (and
       (at start (<= (pay-count ?m) 2)) 
       (at start (token-step ?carrier ?from dispose)) 
       (at start (step-place dispose ?to)) 
       (at start (rs-slide ?m ?to)) 
       (at start (not (at ?carrier ?from)))
     )
     :effect (and
       (at end (increase (pay-count ?m) 1)) 
     )
   )
