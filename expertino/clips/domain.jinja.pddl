(define (domain workpiece_flow)
  ; standalone parser of nextflap crashes when you both have an
  ; instance of an object and a sub-object of the same type!
  ; e.g., "inner - object" and "outer - inner" means that no object should have
  ; type "inner"
  ; This is fixed by using the parser from the unified planning framework
  (:types
    interactable place side task - object
    workpiece machine - interactable
    carrier product payment - workpiece
    shelf-slot - object
    meta base ring cap - task
    base-station cap-station ring-station storage-station delivery-station - machine
    slide bs-place cs-place rs-place ss-place ds-place - place
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
  (:predicates
     ;; Locations of workpieces
     (at ?wp - workpiece ?p - place) ; Workpiece is at a machine side
 
     ;; Workpiece tasks
     (step ?wp - workpiece ?r - task) ; Current active task
     (next-step ?wp - workpiece ?task1 - task ?task2 - task) ; Requirement order
 
     ;; Machine capabilities
     (step-place ?task - task ?p - place) ; Machine can process a task
 
     ;; Side availability
     (free ?p - place)
     (spawnable ?wp - workpiece)
     (usable ?i - interactable)
     (in ?m - machine ?p - place)
     (out ?m - machine ?p - place)
 
     (rs-slide ?m - ring-station ?s - slide)

     (buffered ?m - cap-station ?c - cap)
     (can-buffer ?m - cap-station ?c - cap)

     (on-shelf ?c - workpiece ?m - cap-station)
     (next-payment ?curr - payment ?next -payment)
  )
  ; make sure functions are defined after predicates
  (:functions
    (order)
    (price ?r - ring)
    (pay-count ?rs - ring-station)
  )
{% block actions %}

{% endblock %}
)

