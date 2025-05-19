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

;; Actions needed for high-level planning

   ;; Dispense a workpiece to the initial location
   (:durative-action spawn-and-transport
     :parameters (?prod - product ?to - place ?step - step-name ?next - step-name)
     :duration (= ?duration 5)
     :condition (and
       (at start (spawnable ?prod))
       (at start (step ?prod ?step))
       (at start (next-step ?prod ?step ?next))
       (at start (step-place ?next ?to))
       (at start (free ?to))
       (over all (free ?to))
     )
     :effect (and
       (at start (not (spawnable ?prod)))
       (at end (at ?prod ?to))
       (at end (not (free ?to)))
       (at end (usable ?prod))
       (at end (not (step ?prod ?step)))
       (at end (step ?prod ?next))
     )
   )

   ;; Transport a workpiece from one machine side to another
   (:durative-action transport
     :parameters (?prod - product ?from - place ?to - place ?step - step-name)
     :duration (= ?duration 5)
     :condition (and
       (at start (at ?prod ?from))
       (at start (step ?prod ?step))
       (at start (step-place ?step ?to))
       (at start (free ?to))
       (at start (usable ?prod))
       (at start (not (= ?from ?to)))
       (over all (free ?to))
     )
     :effect (and
       (at start (not (usable ?prod)))
       (at end (at ?prod ?to))
       (at end (not (free ?to)))
       (at end (free ?from))
       (at end (usable ?prod))
       (at end (not (at ?prod ?from)))
     )
   )

   (:durative-action pay-with-carrier
     :parameters (?m - ring-station ?carrier - carrier ?from - place ?to - slide)
     :duration (= ?duration 5)
     :condition (and
       (at start (at ?carrier ?from))
       (at start (token-usable ?carrier ?from))
       (at start (token-step ?carrier ?from dispose))
       (at start (step-place dispose ?to))
       (at start (<= (pay-count ?m) 2))
     )
     :effect (and
       (at start (not (token-usable ?carrier ?from)))
       (at end (free ?from))
       (at end (increase (pay-count ?m) 1))
       (at end (not (at ?carrier ?from)))
     )
   )

   (:durative-action pay-from-bs
     :parameters (?rs - ring-station)
     :duration (= ?duration 5)
     :condition (and
       (at start (<= (pay-count ?rs) 2))
     )
     :effect (and
       (at end (increase (pay-count ?rs) 1))
     )
   )


   ;; Process a workpiece for its current step-name
   (:durative-action cs-mount-cap
     :parameters (?prod - product ?cs - cap-station ?in - place ?out - place ?step - cap ?next - step-name)
     :duration (= ?duration 10)
     :condition (and
       (at start (at ?prod ?in))
       (at start (step ?prod ?step))
       (at start (next-step ?prod ?step ?next))
       (at start (in ?cs ?in))
       (at start (step-place ?step ?in))
       (at start (buffered ?cs ?step))
       (at start (out ?cs ?out))
       (over all (usable ?cs))
       (at start (usable ?prod))
       (at start (free ?out))
     )
     :effect (and
       (at start (not (usable ?prod)))
       (at end (usable ?prod))
       (at end (not (step ?prod ?step)))
       (at end (step ?prod ?next))
       (at end (free ?in))
       (at end (not (buffered ?cs ?step)))
       (at end (can-buffer ?cs ?step))
       (at end (not (free ?out)))
       (at end (not (at ?prod ?in)))
       (at end (at ?prod ?out))
     )
   )

   (:durative-action cs-buffer
     :parameters (?carrier - carrier ?cs - cap-station ?in - place ?out - place ?step - cap)
     :duration (= ?duration 10)
     :condition (and
       (at start (at ?carrier ?in))
       (at start (out ?cs ?out))
       (at start (token-step ?carrier ?in ?step))
       (at start (token-step ?carrier ?out dispose))
       (at start (in ?cs ?in))
       (at start (step-place ?step ?in))
       (at start (can-buffer ?cs ?step))
       (over all (usable ?cs))
       (at start (token-usable ?carrier ?in))
       (at start (free ?out))
     )
     :effect (and
       (at start (not (token-usable ?carrier ?in)))
       (at start (not (can-buffer ?cs ?step)))
       (at end (token-usable ?carrier ?out))
       (at end (free ?in))
       (at end (buffered ?cs ?step))
       (at end (not (free ?out)))
       (at end (not (at ?carrier ?in)))
       (at end (at ?carrier ?out))
     )
   )

   (:durative-action carrier-to-input
     :parameters (?carrier - carrier ?step - cap  ?cs - cap-station ?in - place)
     :duration (= ?duration 10)
     :condition (and
       (at start (in ?cs ?in))
       (at start (token-step ?carrier ?in ?step))
       (at start (free ?in))
       (at start (step-place ?step ?in))
       (at start (can-buffer ?cs ?step))
       (at start (on-shelf ?carrier ?cs))
       (over all (free ?in))
     )
     :effect (and
       (at end (token-usable ?carrier ?in))
       (at end (at ?carrier ?in))
       (at end (not (free ?in)))
     )
   )

   (:durative-action rs-mount-ring
     :parameters (?prod - product ?rs - ring-station ?in - place ?out - place ?step - ring ?next - step-name)
     :duration (= ?duration 10)
     :condition (and
       (at start (at ?prod ?in))
       (at start (step ?prod ?step))
       (at start (next-step ?prod ?step ?next))
       (at start (in ?rs ?in))
       (at start (>= (pay-count ?rs) (price ?step)))
       (at start (step-place ?step ?in))
       (at start (out ?rs ?out))
       (over all (usable ?rs))
       (at start (usable ?prod))
       (at start (free ?out))
     )
     :effect (and
       (at start (not (usable ?prod)))
       (at end (decrease (pay-count ?rs) (price ?step)))
       (at end (usable ?prod))
       (at end (not (step ?prod ?step)))
       (at end (step ?prod ?next))
       (at end (free ?in))
       (at end (not (free ?out)))
       (at end (not (at ?prod ?in)))
       (at end (at ?prod ?out))
     )
   )

   ;; Final delivery
   (:durative-action finalize
     :parameters (?prod - product ?m - machine ?in - place ?step - step-name)
     :duration (= ?duration 10)
     :condition (and
       (at start (at ?prod ?in))
       (at start (step ?prod ?step))
       (at start (next-step ?prod ?step done))
       (at start (in ?m ?in))
       (at start (step-place ?step ?in))
       (at start (usable ?m))
       (at start (usable ?prod))
     )
     :effect (and
       (at start (not (usable ?m)))
       (at start (not (usable ?prod)))
       (at end (usable ?m))
       ;(at end (usable ?prod))
       (at end (not (step ?prod ?step)))
       (at end (step ?prod done))
       (at end (free ?in))
       (at end (not (at ?prod ?in)))
     )
   )

