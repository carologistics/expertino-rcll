(define (domain workpiece_flow)

  (:types
    workpiece place side req machine - object
    carrier product payment - workpiece
    shelfSlot - object
    meta base ring cap - req
    baseStation capStation ringStation storageStation deliveryStation - machine
    slide bsPlace csPlace rsPlace ssPlace dsPlace - place
  )
  (:constants 
    BS - baseStation
    CS1 CS2 - capStation
    RS1 RS2 - ringStation
    SS - storageStation
    DS - deliveryStation
    BSInput BSOutput - bsPlace
    CS1Input CS1Output CS2Input CS2Output - csPlace
    RS1Slide RS2Slide - slide
    RS1Input RS1Output RS2Input RS2Output - rsPlace
    SSInput SSOutput - ssPlace
    DSInput - dsPlace
    PAY DELIVER DONE - meta
    BASE_RED BASE_SILVER BASE_BLACK - base
    RING_BLUE1 RING_YELLOW1 RING_GREEN1 RING_ORANGE1 RING_BLUE2 RING_YELLOW2 RING_GREEN2 RING_ORANGE2 RING_BLUE3 RING_YELLOW3 RING_GREEN3 RING_ORANGE3 - ring
    ;CAP_GREY CAP_BLACK - req
    CAP_GREY CAP_BLACK - cap
  )
  (:predicates
     ;; Locations of workpieces
     (at ?wp - workpiece ?p - place) ; Workpiece is at a machine side
 
     ;; Workpiece reqs
     (currentRequirement ?wp - workpiece ?r - req) ; Current active req
     (nextRequirement ?wp - workpiece ?req1 - req ?req2 - req) ; Requirement order
 
     ;; Machine capabilities
     (processPlace ?p - place ?req - req) ; Machine can process a req
 
     ;; Side availability
     (free ?p - place)
     (spawnable ?wp - workpiece)
     (available ?m - machine)
     (blocked ?p - place)
     (usable ?wp - workpiece)
     (in ?m - machine ?p - place)
     (out ?m - machine ?p - place)
 
 	(pay-at ?m - ringStation ?s - slide)
 
 	(buffered ?m - capStation ?c - req)
 	(can-buffer ?m - capStation ?c - req)
 
 	(on-shelf ?c - workpiece ?m - capStation)
  )
  (:functions
    (pay-req ?r - ring)
    (pay-count ?rs - ringStation)
  )

   ;; Dispense a workpiece to the initial location
   (:durative-action dispense
     :parameters (?wp - product ?m - baseStation ?p - bsPlace ?req - req ?next - req)
     :duration (= ?duration 5)
     :condition (and
       (at start (spawnable ?wp))
       (at start (free BSInput))
       (at start (free BSOutput))
       (at start (currentRequirement ?wp ?req))
       (at start (nextRequirement ?wp ?req ?next))
       (at start (available ?m))
     )
     :effect (and
       (at start (not (spawnable ?wp)))
       (at start (not (available ?m)))
       (at end (available ?m))
       (at end (usable ?wp))
       (at end (at ?wp ?p))
       (at end (not (currentRequirement ?wp ?req)))
       (at end (currentRequirement ?wp ?next))
       (at end (not (free ?p)))
     )
   )
   (:durative-action dispensePay
     :parameters (?wp - payment ?m - baseStation ?p - bsPlace)
     :duration (= ?duration 5)
     :condition (and
       (at start (spawnable ?wp))
       (at start (currentRequirement ?wp PAY))
       (at start (free BSInput))
       (at start (free BSOutput))
       (at start (available ?m))
     )
     :effect (and
       (at start (not (spawnable ?wp)))
       (at start (not (available ?m)))
       (at end (available ?m))
       (at end (usable ?wp))
       (at end (at ?wp ?p))
       (at end (not (free ?p)))
     )
   )
 
   ;; Transport a workpiece from one machine side to another
   (:durative-action transport
     :parameters (?wp - workpiece ?from - place ?to - place ?r - req)
     :duration (= ?duration 5)
     :condition (and
       (at start (at ?wp ?from))
       (at start (currentRequirement ?wp ?r))
       (at start (processPlace ?to ?r))
       (at start (free ?to))
       (at start (usable ?wp))
       (over all (free ?to))
     )
     :effect (and
       (at start (not (usable ?wp)))
       (at end (at ?wp ?to))
       (at end (not (free ?to)))
       (at end (free ?from))
       (at end (usable ?wp))
       (at end (not (at ?wp ?from)))
     )
   )
 
   ;; Process a workpiece for its current req
   (:durative-action processCSMount
     :parameters (?wp - product ?m - capStation ?in - place ?out - place ?req - cap ?next - req)
     :duration (= ?duration 10)
     :condition (and
       (at start (at ?wp ?in))
       (at start (currentRequirement ?wp ?req))
       (at start (nextRequirement ?wp ?req ?next))
       (at start (in ?m ?in))
       (at start (processPlace ?in ?req))
       (at start (buffered ?m ?req))
       (at start (out ?m ?out))
       (over all (available ?m))
       (at start (usable ?wp))
       (at start (free ?out))
     )
     :effect (and
       (at start (not (usable ?wp)))
       (at end (usable ?wp))
       (at end (not (currentRequirement ?wp ?req)))
       (at end (currentRequirement ?wp ?next))
       (at end (free ?in))
       (at end (not (buffered ?m ?req)))
       (at end (can-buffer ?m ?req))
       (at end (not (free ?out)))
       (at end (not (at ?wp ?in)))
       (at end (at ?wp ?out))
     )
   )
   (:durative-action processCSBuffer
     :parameters (?wp - carrier ?m - capStation ?in - place ?out - place ?req - cap)
     :duration (= ?duration 10)
     :condition (and
       (at start (at ?wp ?in))
       (at start (currentRequirement ?wp ?req))
       (at start (nextRequirement ?wp ?req PAY))
       (at start (in ?m ?in))
       (at start (processPlace ?in ?req))
       (at start (can-buffer ?m ?req))
       (at start (out ?m ?out))
       (over all (available ?m))
       (at start (usable ?wp))
       (at start (free ?out))
     )
     :effect (and
       (at start (not (usable ?wp)))
       (at end (usable ?wp))
       (at end (not (currentRequirement ?wp ?req)))
       (at end (currentRequirement ?wp PAY))
       (at end (free ?in))
       (at end (not (can-buffer ?m ?req)))
       (at end (buffered ?m ?req))
       (at end (not (free ?out)))
       (at end (not (at ?wp ?in)))
       (at end (at ?wp ?out))
     )
   )
   (:durative-action carrierToInput
     :parameters (?wp - carrier ?req - cap  ?m - capStation ?in - place)
     :duration (= ?duration 10)
     :condition (and
       (at start (currentRequirement ?wp ?req))
       (at start (in ?m ?in))
       (at start (free ?in))
       (at start (usable ?wp))
       (at start (processPlace ?in ?req))
       (at start (can-buffer ?m ?req))
       (at start (on-shelf ?wp ?m))
     )
     :effect (and
       (at start (not (usable ?wp)))
       (at end (usable ?wp))
       (at end (at ?wp ?in))
       (at end (not (free ?in)))
       (at end (not (on-shelf ?wp ?m)))
     )
   )
   (:durative-action processRS
     :parameters (?wp - product ?m - ringStation ?in - place ?out - place ?req - ring ?next - req)
     :duration (= ?duration 10)
     :condition (and
       (at start (at ?wp ?in))
       (at start (currentRequirement ?wp ?req))
       (at start (nextRequirement ?wp ?req ?next))
       (at start (in ?m ?in))
       (at start (>= (pay-count ?m) (pay-req ?req)))
       (at start (processPlace ?in ?req))
       (at start (out ?m ?out))
       (over all (available ?m))
       (at start (usable ?wp))
       (at start (free ?out))
     )
     :effect (and
       (at start (not (usable ?wp)))
       (at end (decrease (pay-count ?m) (pay-req ?req)))
       (at end (usable ?wp))
       (at end (not (currentRequirement ?wp ?req)))
       (at end (currentRequirement ?wp ?next))
       (at end (free ?in))
       (at end (not (free ?out)))
       (at end (not (at ?wp ?in)))
       (at end (at ?wp ?out))
     )
   )
   ;; Finalize a workpiece
   (:durative-action finalize
     :parameters (?wp - product ?m - machine ?in - place ?req - req)
     :duration (= ?duration 10)
     :condition (and
       (at start (at ?wp ?in))
       (at start (currentRequirement ?wp ?req))
       (at start (nextRequirement ?wp ?req DONE))
       (at start (in ?m ?in))
       (at start (processPlace ?in ?req))
       (at start (available ?m))
       (at start (usable ?wp))
     )
     :effect (and
       (at start (not (available ?m)))
       (at start (not (usable ?wp)))
       (at end (available ?m))
       ;(at end (usable ?wp))
       (at end (not (currentRequirement ?wp ?req)))
       (at end (currentRequirement ?wp DONE))
       (at end (free ?in))
       (at end (not (at ?wp ?in)))
     )
   )
 
   ;; Commit Payment
   (:durative-action pay-at-rs
     :parameters (?wp - product ?req - ring ?pay - payment ?m - ringStation ?slide - slide)
     :duration (= ?duration 0.5)
     :condition (and
       (at start (currentRequirement ?pay PAY))
       (at start (at ?pay ?slide))
       (at start (pay-at ?m ?slide))
       (at start (usable ?pay))
       (at start (available ?m))
       (at start (<= (pay-count ?m) 2))
       (at start (currentRequirement ?wp ?req))
       (at start (>= (pay-count ?m) (pay-req ?req)))
     )
     :effect (and
       (at start (not (available ?m)))
       (at start (not (usable ?pay)))
       (at end (not (at ?pay ?slide)))
       (at end (free ?slide))
       (at end (available ?m))
       (at end (increase (pay-count ?m) 1))
     )
   )
   ;; Commit Payment
   (:durative-action pay-wit-cc-at-rs
     :parameters (?wp - product ?curr - req ?next - cap ?pay - carrier ?m - ringStation ?slide - slide)
     :duration (= ?duration 0.5)
     :condition (and
       (at start (currentRequirement ?wp ?curr))
       (at start (nextRequirement ?wp ?curr ?next))
       (at start (currentRequirement ?pay PAY))
       (at start (at ?pay ?slide))
       (at start (pay-at ?m ?slide))
       (at start (usable ?pay))
       (at start (available ?m))
       (at start (<= (pay-count ?m) 2))
     )
     :effect (and
       (at start (not (available ?m)))
       (at start (not (usable ?pay)))
       (at end (not (at ?pay ?slide)))
       (at end (available ?m))
       (at end (increase (pay-count ?m) 1))
     )
   )
 
)

