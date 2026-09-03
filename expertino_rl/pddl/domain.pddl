
(define (domain workpiece-flow)

(:requirements :strips :typing :equality :numeric-fluents :negative-preconditions :durative-actions)

(:types
  interactable place side step-name - object
  workpiece machine - interactable
  ;; machines
  base-station cap-station ring-station storage-station delivery-station - machine
  slide bs-place cs-place rs-place ss-place ds-place - place

  ;; moveable entities
  ; products are named entities (1 pddl object per physical object)
	; tokens are unnamed entitites (1 pddl object for N pysical objects)
  token product - workpiece
  carrier payment - token
  meta base ring cap - step-name
)

(:constants 
  bs - base-station
  cs1 cs2 - cap-station
  rs1 rs2 - ring-station
  ss - storage-station
  ds - delivery-station

  bs-input bs-output - bs-place
  cs1-input cs1-output cs2-input cs2-output - cs-place
  rs1-slide rs2-slide - slide
  rs1-input rs1-output rs2-input rs2-output - rs-place
  ss-input ss-output - ss-place
  ds-input - ds-place
  dispose deliver done - meta
  base-red base-silver base-black - base
  ring-blue1 ring-yellow1 ring-green1 ring-orange1 ring-blue2 ring-yellow2 ring-green2 ring-orange2 ring-blue3 ring-yellow3 ring-green3 ring-orange3 - ring
  cap-grey cap-black - cap
)

(:predicates ;todo: define predicates here
  ;; Locations of workpieces
  (at ?wp - workpiece ?place - place) ; Workpiece is at a machine side

  ;; Workpiece step-names
  (step ?prod - product ?step - step-name) ; Current active step-name
  (next-step ?prod - product ?curr-step - step-name ?next-step - step-name) ; Requirement order
  (token-step ?token - token ?place - place ?step - step-name)

  ;; Machine capabilities
  (step-place ?step - step-name ?place - place) ; Machine can process a step

  ;; Side availability
  (free ?place - place)
  (spawnable ?product - product)
  (usable ?i - interactable)
  (in ?m - machine ?place - place)
  (out ?m - machine ?place - place)
  ; specific version of usable to distinguish tokens at different places
  (token-usable ?token - token ?place - place)
  (on-shelf ?carrier - carrier ?cs - cap-station)

  (rs-slide ?rs - ring-station ?slide - slide)

  (buffered ?cs - cap-station ?cap - cap)
  (can-buffer ?cs - cap-station ?cap - cap)
)


(:functions
  (price ?ring - ring)
  (pay-count ?rs - ring-station)
)

(:durative-action transport
    :parameters (?prod - product ?from - place ?to - place ?step - step-name)
    :duration (= ?duration 5)
    :condition (and 
      (at start (at ?prod ?from))
      (at start (free ?to))
      (at start (not (= ?from ?to)))
      (at start (step ?prod ?step))
      (at start (step-place ?step ?to))
      (at start (usable ?prod))
      (over all (free ?to))
    )
    :effect (and 
      (at start (not (usable ?prod)))
      (at start (not (free ?to)))
      (at end (usable ?prod))
      (at end (at ?prod ?to))
      (at end (not (at ?prod ?from)))
      (at end (free ?from))
    )
)

(:durative-action transport-from-rs-to-cs
    :parameters (?prod - product ?from - rs-place ?cs - cap-station ?in - cs-place ?step - cap)
    :duration (= ?duration 5)
    :condition (and 
      (at start (at ?prod ?from))
      (at start (in ?cs ?in))
      (at start (buffered ?cs ?step))
      (at start (free ?in))
      (at start (step ?prod ?step))
      (at start (usable ?prod))
      (over all (free ?in))
    )
    :effect (and
      (at start (not (usable ?prod)))
      (at start (not (free ?in)))
      (at end (usable ?prod))
      (at end (at ?prod ?in))
      (at end (not (at ?prod ?from)))
      (at end (free ?from)) 
    )
)

(:durative-action transport-from-bs
    :parameters (?prod - product ?bs - base-station ?from - bs-place ?to - place ?step - step-name)
    :duration (= ?duration 5)
    :condition (and 
      (at start (at ?prod ?from))
      (at start (free ?to))
      (at start (not (= ?from ?to)))
      (at start (step ?prod ?step))
      (at start (step-place ?step ?to))
      (at start (usable ?prod))
      (over all (free ?to))
    )
    :effect (and 
      (at start (not (usable ?prod)))
      (at start (not (free ?to)))
      (at end (usable ?prod))
      (at end (at ?prod ?to))
      (at end (not (at ?prod ?from)))
      (at end (free ?from))
      (at end (usable ?bs))
    )
)

(:durative-action transport-from-bs-to-cs
    :parameters (?prod - product ?bs - base-station ?from - bs-place ?cs - cap-station ?in - cs-place ?step - cap)
    :duration (= ?duration 5)
    :condition (and 
      (at start (at ?prod ?from))
      (at start (in ?cs ?in))
      (at start (buffered ?cs ?step))
      (at start (free ?in))
      (at start (step ?prod ?step))
      (at start (usable ?prod))
      (over all (free ?in))
    )
    :effect (and
      (at start (not (usable ?prod)))
      (at start (not (free ?in)))
      (at end (usable ?prod))
      (at end (at ?prod ?in))
      (at end (not (at ?prod ?from)))
      (at end (free ?from))
      (at end (usable ?bs))
    )
)

(:durative-action bs-dispense
    :parameters (?prod - product ?bs - base-station ?p - bs-place ?step - step-name ?next - step-name)
    :duration (= ?duration 10)
    :condition (and 
      (at start (spawnable ?prod))
      (at start (free ?p))
      (at start (step ?prod ?step))
      (at start (next-step ?prod ?step ?next))
      (at start (usable ?bs))
    )
    :effect (and 
      (at start (not (spawnable ?prod)))
      (at start (not (usable ?bs)))
      (at end (at ?prod ?p))
      (at end (not (free ?p)))
      (at end (not (step ?prod ?step)))
      (at end (step ?prod ?next))
      (at end (usable ?prod))
    )
)

(:durative-action carrier-to-input
    :parameters (?carrier - carrier ?step - cap ?cs - cap-station ?in - cs-place)
    :duration (= ?duration 10)
    :condition (and 
      (at start (in ?cs ?in))
      (at start (free ?in))
      (at start (can-buffer ?cs ?step))
      (at start (on-shelf ?carrier ?cs))
      (at start (free ?in))
      (at start (token-step ?carrier ?in ?step))
      (at start (step-place ?step ?in))
      (over all (free ?in))
    )
    :effect (and 
      (at start (not (free ?in)))
      (at end (at ?carrier ?in))
      (at end (token-usable ?carrier ?in))
    )
)

(:durative-action cs-buffer
    :parameters (?carrier - carrier ?cs - cap-station ?in - cs-place ?out - cs-place ?step - cap)
    :duration (= ?duration 10)
    :condition (and 
      (at start (at ?carrier ?in))
      (at start (out ?cs ?out))
      (at start (in ?cs ?in))
      (at start (step-place ?step ?in))
      (at start (can-buffer ?cs ?step))
      (at start (free ?out))
      (at start (token-step ?carrier ?in ?step))
      (at start (token-step ?carrier ?out dispose))
      (at start (token-usable ?carrier ?in))
      (over all (usable ?cs))
    )
    :effect (and 
      (at start (not (can-buffer ?cs ?step)))
      (at start (not (free ?out)))
      (at start (not (token-usable ?carrier ?in)))
      (at end (free ?in))
      (at end (buffered ?cs ?step))
      (at end (not (at ?carrier ?in)))
      (at end (at ?carrier ?out))
      (at end (token-usable ?carrier ?out))
    )
)

(:durative-action cs-mount-cap
    :parameters (?prod - product ?cs - cap-station ?in - place ?out - place ?step - cap ?next - step-name)
    :duration (= ?duration 10)
    :condition (and 
      (at start (at ?prod ?in))
      (at start (in ?cs ?in))
      (at start (out ?cs ?out))
      (at start (step-place ?step ?in))
      (at start (buffered ?cs ?step))
      (at start (free ?out))
      (at start (step ?prod ?step))
      (at start (next-step ?prod ?step ?next))
      (at start (usable ?prod))
      (over all (usable ?cs))
    )
    :effect (and 
      (at start (not (usable ?prod)))
      (at start (not (free ?out)))
      (at end (free ?in))
      (at end (not (buffered ?cs ?step)))
      (at end (can-buffer ?cs ?step))
      (at end (not (at ?prod ?in)))
      (at end (at ?prod ?out))
      (at end (not (step ?prod ?step)))
      (at end (step ?prod ?next))
      (at end (usable ?prod))
    )
)

(:durative-action pay-with-carrier
    :parameters (?carrier - carrier ?from - place ?rs - ring-station ?to - slide)
    :duration (= ?duration 10)
    :condition (and 
      (at start (at ?carrier ?from))
      (at start (rs-slide ?rs ?to))
      (at start (token-usable ?carrier ?from))
      (at start (token-step ?carrier ?from dispose))
      (at start (step-place dispose ?to))
      (at start (<= (pay-count ?rs) 2))
    )
    :effect (and 
      (at start (not (token-usable ?carrier ?from)))
      (at end (free ?from))
      (at end (increase (pay-count ?rs) 1))
      (at end (not (at ?carrier ?from)))
    )
)

(:durative-action bs-dispense-pay
    :parameters (?token - payment ?bs - base-station ?from - bs-place)
    :duration (= ?duration 10)
    :condition (and 
      (at start (usable ?bs))
      (at start (token-step ?token ?from dispose))
      (at start (free ?from))
    )
    :effect (and 
      (at start (not (free ?from)))
      (at start (not (usable ?bs)))
      (at end (token-usable ?token ?from))
      (at end (at ?token ?from))
    )
)

(:durative-action pay-with-base
    :parameters (?token - payment ?bs - base-station ?from - bs-place ?rs - ring-station ?to - slide)
    :duration (= ?duration 10)
    :condition (and 
      (at start (<= (pay-count ?rs) 2))
      (at start (at ?token ?from))
      (at start (token-usable ?token ?from))
      (at start (token-step ?token ?from dispose))
      (at start (rs-slide ?rs ?to))
    )
    :effect (and 
      (at end (usable ?bs))
      (at end (not (token-usable ?token ?from)))
      (at end (increase (pay-count ?rs) 1))
      (at end (free ?from))
      (at end (not (at ?token ?from)))
    )
)

(:durative-action rs-mount-ring
    :parameters (?prod - product ?rs - ring-station ?in - rs-place ?out - rs-place ?step - ring ?next - step-name)
    :duration (= ?duration 10)
    :condition (and 
      (at start (at ?prod ?in))
      (at start (in ?rs ?in))
      (at start (out ?rs ?out))
      (at start (>= (pay-count ?rs) (price ?step)))
      (at start (step-place ?step ?in))
      (at start (free ?out))
      (at start (step ?prod ?step))
      (at start (next-step ?prod ?step ?next))
      (at start (usable ?prod))
    )
    :effect (and 
      (at start (not (free ?out)))
      (at start (not (usable ?prod)))
      (at end (decrease (pay-count ?rs) (price ?step)))
      (at end (free ?in))
      (at end (not (at ?prod ?in)))
      (at end (at ?prod ?out))
      (at end (not (step ?prod ?step)))
      (at end (step ?prod ?next))
      (at end (usable ?prod))
    )
)

(:durative-action finalize
    :parameters (?prod - product ?ds - delivery-station ?in - ds-place ?step - step-name)
    :duration (= ?duration 10)
    :condition (and 
      (at start (at ?prod ?in))
      (at start (in ?ds ?in))
      (at start (step-place ?step ?in))
      (at start (step ?prod ?step))
      (at start (next-step ?prod ?step done))
      (at start (usable ?ds))
      (at start (usable ?prod))
    )
    :effect (and 
      (at start (not (usable ?ds)))
      (at start (not (usable ?prod)))
      (at end (not (step ?prod ?step)))
      (at end (step ?prod done))
      (at end (free ?in))
      (at end (not (at ?prod ?in)))
      (at end (usable ?ds))
    )
)









)