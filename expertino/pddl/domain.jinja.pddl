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

{% extends "domain_base.jinja.pddl" %}
{% block actions %}
   ;; Dispense a workpiece to the initial location
   (:durative-action bs-dispense
     :parameters (?wp - product ?m - base-station ?p - bs-place ?task - task ?next - task)
     :duration (= ?duration 5)
     :condition (and
       (at start (spawnable ?wp))
       (at start (free bs-input))
       (at start (free bs-output))
       (at start (step ?wp ?task))
       (at start (next-step ?wp ?task ?next))
       (at start (usable ?m))
     )
     :effect (and
       (at start (not (spawnable ?wp)))
       (at start (not (usable ?m)))
       (at end (usable ?m))
       (at end (usable ?wp))
       (at end (at ?wp ?p))
       (at end (not (step ?wp ?task)))
       (at end (step ?wp ?next))
       (at end (not (free ?p)))
     )
   )
   (:durative-action dispense-pay
     :parameters (?wp - payment ?next - payment ?m - base-station ?p - bs-place)
     :duration (= ?duration 5)
     :condition (and
       (at start (spawnable ?wp))
       (at start (next-payment ?wp ?next))
       (at start (step ?wp dispose))
       (at start (free bs-input))
       (at start (free bs-output))
       (at start (usable ?m))
     )
     :effect (and
       (at start (not (spawnable ?wp)))
       (at start (not (usable ?m)))
       (at end (usable ?m))
       (at end (usable ?wp))
       (at end (at ?wp ?p))
       (at end (spawnable ?next))
       (at end (not (free ?p)))
     )
   )

   ;; Transport a workpiece from one machine side to another
   (:durative-action transport
     :parameters (?wp - workpiece ?from - place ?to - place ?r - task)
     :duration (= ?duration 5)
     :condition (and
       (at start (at ?wp ?from))
       (at start (step ?wp ?r))
       (at start (step-place ?r ?to))
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

   ;; Process a workpiece for its current task
   (:durative-action cs-mount-cap
     :parameters (?wp - product ?m - cap-station ?in - place ?out - place ?task - cap ?next - task)
     :duration (= ?duration 10)
     :condition (and
       (at start (at ?wp ?in))
       (at start (step ?wp ?task))
       (at start (next-step ?wp ?task ?next))
       (at start (in ?m ?in))
       (at start (step-place ?task ?in))
       (at start (buffered ?m ?task))
       (at start (out ?m ?out))
       (over all (usable ?m))
       (at start (usable ?wp))
       (at start (free ?out))
     )
     :effect (and
       (at start (not (usable ?wp)))
       (at end (usable ?wp))
       (at end (not (step ?wp ?task)))
       (at end (step ?wp ?next))
       (at end (free ?in))
       (at end (not (buffered ?m ?task)))
       (at end (can-buffer ?m ?task))
       (at end (not (free ?out)))
       (at end (not (at ?wp ?in)))
       (at end (at ?wp ?out))
     )
   )
   (:durative-action cs-buffer
     :parameters (?wp - carrier ?m - cap-station ?in - place ?out - place ?task - cap)
     :duration (= ?duration 10)
     :condition (and
       (at start (at ?wp ?in))
       (at start (step ?wp ?task))
       (at start (next-step ?wp ?task dispose))
       (at start (in ?m ?in))
       (at start (step-place ?task ?in))
       (at start (can-buffer ?m ?task))
       (at start (out ?m ?out))
       (over all (usable ?m))
       (at start (usable ?wp))
       (at start (free ?out))
     )
     :effect (and
       (at start (not (usable ?wp)))
       (at end (usable ?wp))
       (at end (not (step ?wp ?task)))
       (at end (step ?wp dispose))
       (at end (free ?in))
       (at end (not (can-buffer ?m ?task)))
       (at end (buffered ?m ?task))
       (at end (not (free ?out)))
       (at end (not (at ?wp ?in)))
       (at end (at ?wp ?out))
     )
   )
   (:durative-action carrier-to-input
     :parameters (?wp - carrier ?task - cap  ?m - cap-station ?in - place)
     :duration (= ?duration 10)
     :condition (and
       (at start (step ?wp ?task))
       (at start (in ?m ?in))
       (at start (free ?in))
       (at start (usable ?wp))
       (at start (step-place ?task ?in))
       (at start (can-buffer ?m ?task))
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
   (:durative-action rs-mount-ring
     :parameters (?wp - product ?m - ring-station ?in - place ?out - place ?task - ring ?next - task)
     :duration (= ?duration 10)
     :condition (and
       (at start (at ?wp ?in))
       (at start (step ?wp ?task))
       (at start (next-step ?wp ?task ?next))
       (at start (in ?m ?in))
       (at start (>= (pay-count ?m) (price ?task)))
       (at start (step-place ?task ?in))
       (at start (out ?m ?out))
       (over all (usable ?m))
       (at start (usable ?wp))
       (at start (free ?out))
     )
     :effect (and
       (at start (not (usable ?wp)))
       (at end (decrease (pay-count ?m) (price ?task)))
       (at end (usable ?wp))
       (at end (not (step ?wp ?task)))
       (at end (step ?wp ?next))
       (at end (free ?in))
       (at end (not (free ?out)))
       (at end (not (at ?wp ?in)))
       (at end (at ?wp ?out))
     )
   )
   ;; Finalize a workpiece
   (:durative-action finalize
     :parameters (?wp - product ?m - machine ?in - place ?task - task)
     :duration (= ?duration 10)
     :condition (and
       (at start (at ?wp ?in))
       (at start (step ?wp ?task))
       (at start (next-step ?wp ?task done))
       (at start (in ?m ?in))
       (at start (step-place ?task ?in))
       (at start (usable ?m))
       (at start (usable ?wp))
     )
     :effect (and
       (at start (not (usable ?m)))
       (at start (not (usable ?wp)))
       (at end (usable ?m))
       ;(at end (usable ?wp))
       (at end (not (step ?wp ?task)))
       (at end (step ?wp done))
       (at end (free ?in))
       (at end (not (at ?wp ?in)))
     )
   )

   (:durative-action rs-pay
     :parameters (?wp - product ?task - ring ?pay - payment ?m - ring-station ?slide - slide)
     :duration (= ?duration 0.5)
     :condition (and
       (at start (step ?pay dispose))
       (at start (at ?pay ?slide))
       (at start (rs-slide ?m ?slide))
       (at start (usable ?pay))
       (at start (usable ?m))
       (at start (<= (pay-count ?m) 2))
       (at start (step ?wp ?task))
       (at start (>= (pay-count ?m) (price ?task)))
     )
     :effect (and
       (at start (not (usable ?m)))
       (at start (not (usable ?pay)))
       (at end (not (at ?pay ?slide)))
       (at end (free ?slide))
       (at end (usable ?m))
       (at end (increase (pay-count ?m) 1))
     )
   )

   (:durative-action rs-pay-with-cc
     :parameters (?wp - product ?curr - task ?next - cap ?pay - carrier ?m - ring-station ?slide - slide)
     :duration (= ?duration 0.5)
     :condition (and
       (at start (step ?wp ?curr))
       (at start (next-step ?wp ?curr ?next))
       (at start (step ?pay dispose))
       (at start (at ?pay ?slide))
       (at start (rs-slide ?m ?slide))
       (at start (usable ?pay))
       (at start (usable ?m))
       (at start (<= (pay-count ?m) 2))
     )
     :effect (and
       (at start (not (usable ?m)))
       (at start (not (usable ?pay)))
       (at end (not (at ?pay ?slide)))
       (at end (usable ?m))
       (at end (increase (pay-count ?m) 1))
     )
   )
{% endblock %}
