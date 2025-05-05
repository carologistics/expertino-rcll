; Copyright (c) 2024 Carologistics
;
; Licensed under the Apache License, Version 2.0 (the "License");
; you may not use this file except in compliance with the License.
; You may obtain a copy of the License at
;
;     http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS,
; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.

;; Basic definitions sufficient for high-level planning

(define (domain workpiece_flow)
  ; standalone parser of nextflap crashes when you both have an
  ; instance of an object and a sub-object of the same type!
  ; e.g., "inner - object" and "outer - inner" means that no object should have
  ; type "inner"
  ; This is fixed by using the parser from the unified planning framework
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
{% block types %}
{% endblock %}
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
{% block constants %}
{% endblock %}
  )

  (:predicates
     ;; Locations of workpieces
     (at ?wp - workpiece ?place - place) ; Workpiece is at a machine side

     ;; Workpiece step-names
     (step ?prod - product ?step - step-name) ; Current active step-name
     (next-step ?prod - product ?curr-step - step-name ?next-step - step-name) ; Requirement order
     (token-step ?token - token ?place - place ?step - step-name) ; 

     ;; Machine capabilities
     (step-place ?step - step-name ?place - place) ; Machine can process a step

     ;; Side availability
     (free ?place - place)
     (spawnable ?product - product)
     (usable ?i - interactable) ; do not use this for tokens
     (in ?m - machine ?place - place)
     (out ?m - machine ?place - place)
     ; specific version of usable to distinguish tokens at different places
     (token-usable ?token - token ?place - place)

     (on-shelf ?carrier - carrier ?cs - cap-station)

     (rs-slide ?rs - ring-station ?slide - slide)

     (buffered ?cs - cap-station ?cap - cap)
     (can-buffer ?cs - cap-station ?cap - cap)
{% block predicates %}
{% endblock %}
  )

  ; make sure functions are defined after predicates
  (:functions
    (price ?ring - ring)
    (pay-count ?rs - ring-station)
{% block functions %}
{% endblock %}
  )

{% block actions %}
{% endblock %}
)
