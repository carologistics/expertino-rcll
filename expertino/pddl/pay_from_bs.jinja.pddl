   
   (:durative-action bs-dispense-pay
     :parameters (?token - payment ?bs - base-station ?from - bs-place)
     :duration (= ?duration 5)
     :condition (and
       (at start (not (token-usable ?token ?from)))
       (at start (token-step ?token ?from dispose))
       (at start (usable ?bs)) 
       (at start (free bs-input))
       (at start (free bs-output))
     )
     :effect (and
       (at start (not (usable ?bs)))
       (at end (usable ?bs))
       (at end (token-usable ?token ?from))
       (at end (at ?token ?from))
       (at end (not (free ?from)))
     )
   )
    
   (:durative-action transport-to-slide 
     :parameters (?token - payment ?from - bs-place ?to - slide ?rs - ring-station)  
     :duration (= ?duration 5) 
     :condition (and 
       (at start (<= (pay-count ?rs) 2)) 
       (at start (at ?token ?from)) 
       (at start (step-place dispose ?to)) 
       (at start (token-step ?token ?from dispose)) 
       (at start (token-usable ?token ?from))
       (at start (rs-slide ?rs ?to)) 
     ) 
     :effect (and 
       (at start (not (token-usable ?token ?from))) 
       (at end (increase (pay-count ?rs) 1)) 
       (at end (free ?from)) 
       (at end (not (at ?token ?from))) 
     ) 
   )
