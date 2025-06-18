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

(defrule finalize-ros-destroy-client
" Delete each client on executive finalize. "
  (executive-finalize)
  (ros-msgs-client (service ?service))
=>
  (printout debug "Destroying client" crlf)
  (ros-msgs-destroy-client ?service)
)

(defrule finalize-ros-destroy-message
" Delete each message on executive finalize. "
  (executive-finalize)
  ?msg-f <- (ros-msgs-message (msg-ptr ?ptr))
=>
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
)
(defrule finalize-ros-destroy-response
" Delete each response on executive finalize. "
  (executive-finalize)
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
=>
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
)

(defrule finalize-ros-destroy-plan-temporal-client
" Delete each client on executive finalize. "
  (executive-finalize)
  (pddl-msgs-plan-temporal-client (server ?server))
=>
  (printout debug "Destroying plan-temporal client" crlf)
  (pddl-msgs-plan-temporal-destroy-client ?server)
)
