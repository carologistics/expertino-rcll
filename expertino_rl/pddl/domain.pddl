
(define (domain workpiece-flow)

(:requirements :strips :typing :equality :numeric-fluents :negative-preconditions)

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

(:action transport
    :parameters (?prod - product ?from - place ?to - place ?step - step-name)
    :precondition (and 
      (at ?prod ?from)
      (free ?to)
      (not (= ?from ?to))
      (step ?prod ?step)
      (step-place ?step ?to)
    )
    :effect (and 
      (at ?prod ?to)
      (not (free ?to))
      (not (at ?prod ?from))
      (free ?from)
    )
)


(:action bs-dispense
    :parameters (?prod - product ?p - bs-place ?step - step-name ?next - step-name)
    :precondition (and 
      (spawnable ?prod)
      (free ?p)
      (step ?prod ?step)
      (next-step ?prod ?step ?next)
    )
    :effect (and 
      (not (spawnable ?prod))
      (at ?prod ?p)
      (not (free ?p))
      (not (step ?prod ?step))
      (step ?prod ?next)
    )
)

(:action carrier-to-input
    :parameters (?carrier - carrier ?step - cap ?cs - cap-station ?in - cs-place)
    :precondition (and 
      (in ?cs ?in)
      (free ?in)
      (can-buffer ?cs ?step)
      (on-shelf ?carrier ?cs)
      (free ?in)
      (token-step ?carrier ?in ?step)
      (step-place ?step ?in)
    )
    :effect (and 
      (at ?carrier ?in)
      (not (free ?in))
      (token-usable ?carrier ?in)
    )
)

(:action cs-buffer
    :parameters (?carrier - carrier ?cs - cap-station ?in - cs-place ?out - cs-place ?step - cap)
    :precondition (and 
      (at ?carrier ?in)
      (out ?cs ?out)
      (in ?cs ?in)
      (step-place ?step ?in)
      (can-buffer ?cs ?step)
      (free ?out)
      (token-step ?carrier ?in ?step)
      (token-step ?carrier ?out dispose)
      (token-usable ?carrier ?in)
    )
    :effect (and 
      (not (can-buffer ?cs ?step))
      (free ?in)
      (buffered ?cs ?step)
      (not (free ?out))
      (not (at ?carrier ?in))
      (at ?carrier ?out)
      (not (token-usable ?carrier ?in))
      (token-usable ?carrier ?out)
    )
)

(:action transport-to-cs
    :parameters (?prod - product ?from - place ?cs - cap-station ?in - cs-place ?step - cap)
    :precondition (and 
      (at ?prod ?from)
      (in ?cs ?in)
      (buffered ?cs ?step)
      (free ?in)
    )
    :effect (and 
      (at ?prod ?in)
      (not (free ?in))
      (not (at ?prod ?from))
      (free ?from)
    )
)

(:action cs-mount-cap
    :parameters (?prod - product ?cs - cap-station ?in - place ?out - place ?step - cap ?next - step-name)
    :precondition (and 
      (at ?prod ?in)
      (in ?cs ?in)
      (out ?cs ?out)
      (step-place ?step ?in)
      (buffered ?cs ?step)
      (free ?out)
      (step ?prod ?step)
      (next-step ?prod ?step ?next)
    )
    :effect (and 
      (free ?in)
      (not (free ?out))
      (not (buffered ?cs ?step))
      (can-buffer ?cs ?step)
      (not (at ?prod ?in))
      (at ?prod ?out)
      (not (step ?prod ?step))
      (step ?prod ?next)
    )
)

(:action pay-with-carrier
    :parameters (?carrier - carrier ?from - place ?rs - ring-station ?to - slide)
    :precondition (and 
      (at ?carrier ?from)
      (rs-slide ?rs ?to)
      (token-usable ?carrier ?from)
      (token-step ?carrier ?from dispose)
      (step-place dispose ?to)
      (<= (pay-count ?rs) 2)
    )
    :effect (and 
      (not (token-usable ?carrier ?from))
      (free ?from)
      (increase (pay-count ?rs) 1)
      (not (at ?carrier ?from))
    )
)

(:action bs-dispense-pay
    :parameters (?token - payment ?bs - base-station ?from - bs-place)
    :precondition (and 
      (not (token-usable ?token ?from))
      (token-step ?token ?from dispose)
      (free ?from)
    )
    :effect (and 
      (token-usable ?token ?from)
      (at ?token ?from)
      (not (free ?from))
    )
)

(:action pay-with-base
    :parameters (?token - payment ?from - bs-place ?rs - ring-station ?to - slide)
    :precondition (and 
      (<= (pay-count ?rs) 2)
      (at ?token ?from)
      (token-usable ?token ?from)
      (token-step ?token ?from dispose)
      (rs-slide ?rs ?to)
    )
    :effect (and 
      (not (token-usable ?token ?from))
      (increase (pay-count ?rs) 1)
      (free ?from)
      (not (at ?token ?from))
    )
)

(:action rs-mount-ring
    :parameters (?prod - product ?rs - ring-station ?in - rs-place ?out - rs-place ?step - ring ?next - step-name)
    :precondition (and 
      (at ?prod ?in)
      (in ?rs ?in)
      (out ?rs ?out)
      (>= (pay-count ?rs) (price ?step))
      (step-place ?step ?in)
      (free ?out)
      (step ?prod ?step)
      (next-step ?prod ?step ?next)
    )
    :effect (and 
      (decrease (pay-count ?rs) (price ?step))
      (free ?in)
      (not (free ?out))
      (not (at ?prod ?in))
      (at ?prod ?out)
      (not (step ?prod ?step))
      (step ?prod ?next)
    )
)

(:action finalize
    :parameters (?prod - product ?ds - delivery-station ?in - ds-place ?step - step-name)
    :precondition (and 
      (at ?prod ?in)
      (in ?ds ?in)
      (step-place ?step ?in)
      (step ?prod ?step)
      (next-step ?prod ?step done)
    )
    :effect (and 
      (not (step ?prod ?step))
      (step ?prod done)
      (free ?in)
      (not (at ?prod ?in))
    )
)









)