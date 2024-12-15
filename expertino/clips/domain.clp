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

; TODO: how to initialize domain, should we do it via pddl file or clips?

; TODO: how to handle asynchronous updates?
; e.g., we have list of effects we want to sense feedback
; once feedback comes back we want to update a fluent but we need to wait for feedback to arive before we can actually change it
; we should not do stupid stuff in the meantime with the fluent
; this mainly means we should not evaluate preconditions while this is happening.

; TODO: how to handle action effects and execution
; we kinda need a mapping that can map feedback of a grounded action to the ungrounded action
; e.g., goto R A B gives feedback target-reached that needs to be mapped to some effects add R B, not at R A
; idea is to maintain a set of facts that describes pending effects.
; These are applied either automatically at the relative time points (START, END), or manually through feedback.
; An action can only be done once there are no pending effect facts
; Do we care about races between actions? Probably not (i assume we never did so far)
; Do we care about sanity checks (e.g., delete fluent as effect, but it is already gone, or add fluent but it is already added)?

; TODO: transform TimeTriggeredPlan to PoP in python? Are there algorithms?



(deftemplate pddl-fluent-apply-effect
  (slot fluent-name (type SYMBOL))
  (slot automatic-apply (type SYMBOL) (allowed-values FALSE TRUE) (default FALSE))
)

(deftemplate pddl-fluent-effect-pending
" Describes that a pddl-fluent should be asserted or retracted when the effect is applied"
  (slot action-id (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL))
  (slot value (type SYMBOL) (allowed-values FALSE TRUE) (default FALSE))
  (slot when (type SYMBOL) (allowed-values START END) (default END))
)

(deftemplate pddl-numeric-fluent-effect-pending
" Describes that a pddl-numeric-fluent should be modified when the effect is applied"
  (slot action-id (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL))
  (slot mode (type SYMBOL) (allowed-values ASSIGN INCREMENT DECREMENT))
  (slot value (type FLOAT))
  (slot when (type SYMBOL) (allowed-values START END) (default END))
)

(deftemplate action-feedback
  (slot id (type SYMBOL))
  (slot sub-action (type SYMBOL))
)
