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


(defrule executor-accepted
  (executor (id ?ex-id) (state ACCEPTED) (action-id ?action-id))
  (pddl-action (id ?action-id) (params $?action-params))
  =>
  (assert (pddl-action-get-effect (action ?action-id) (apply TRUE) (effect-type START)))
)

(defrule executor-retract-sending
  ?sending <- (sending ?action-id ?mps)
  (or
    (executor (action-id ?action-id) (state ?s&:(neq ?s ACCEPTED)))
    (not (executor (action-id ?action-id)))
  )
  =>
  (retract ?sending)
  (assert (sent ?action-id))
)


