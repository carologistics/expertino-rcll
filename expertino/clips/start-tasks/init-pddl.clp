
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

; A start-task routine to initialize pddl using the pddl_manager
; Parts:
;  - init-clients: start all service clients
;  - init-problem: setup an initial pddl domain
;  - init-fluents: retrieve fluents from pddl domain and store them
;  - init-functions: retrieve numeric fluents from pddl domain and store them
;

(deffacts pddl-task
  (start-task (name pddl)
    (wait-for)
    (parts init-clients init-problem init-fluents init-functions)
  )
)

(defrule pddl-init-pddl-manager-services
" Create publisher for ros_cx_out."
  (confval (path "/pddl/manager_node") (value ?node))
  ?st <- (start-task
    (name pddl) (state ACTIVE)
    (parts init-clients $?rest-parts)
  )
  (not (executive-finalize))
=>
  ; create all clients
  (bind ?services (create$
    add_fluent AddFluent
    add_pddl_instance AddPddlInstance
    check_action_precondition CheckActionPrecondition
    get_action_effects GetActionEffects
    get_fluents GetFluents
    get_functions GetFunctions
  ))
  (bind ?index 1)
  (bind ?length (length$ ?services))
  (while (< ?index ?length)
     (bind ?service-name (nth$ ?index ?services))
     (bind ?service-type (nth$ (+ ?index 1) ?services))
     (ros-msgs-create-client (str-cat ?node "/" ?service-name) (str-cat "expertino_msgs/srv/" ?service-type))
     (bind ?index (+ ?index 2))
  )
  (modify ?st (parts $?rest-parts))
)

(defrule pddl-request-load-problem-instance
  (confval (path "/pddl/manager_node") (value ?node))
  (confval (path "/pddl/problem_instance") (value ?instance))
  (confval (path "/pddl/pddl_dir") (value ?dir))
  (confval (path "/pddl/init_domain_file") (value ?domain))
  (confval (path "/pddl/init_problem_file") (value ?problem))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/add_pddl_instance"))) (type ?type))
  (not (service-request-meta (service ?s)))
  (start-task (name pddl) (state ACTIVE) (parts init-problem $?rest-parts))
  (time ?any-time) ; used to continuously attempt to request the service until success
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (ros-msgs-set-field ?new-req "name" ?instance)
  (bind ?share-dir (ament-index-get-package-share-directory "expertino"))
  (ros-msgs-set-field ?new-req "directory" (str-cat ?share-dir "/" ?dir))
  (ros-msgs-set-field ?new-req "domain_file" ?domain)
  (ros-msgs-set-field ?new-req "problem_file" ?problem)
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (assert (service-request-meta (service ?s) (request-id ?id) (meta rcll)))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule pddl-init-problem-response-received
" Get response, make sure that it succeeded and delete it afterwards."
  (confval (path "/pddl/manager_node") (value ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/add_pddl_instance"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?) (request-id ?id) (meta ?meta))
  ?st <- (start-task (name pddl) (state ACTIVE) (parts init-problem $?rest-parts))
=>
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (modify ?st (parts $?rest-parts))
   else
    (printout error "Failed to set problem instance \"" ?meta "\":" ?error crlf)
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)

(defrule pddl-request-load-fluents
  (confval (path "/pddl/manager_node") (value ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_fluents"))) (type ?type))
  (not (service-request-meta (service ?s)))
  (start-task (name pddl) (state ACTIVE) (parts init-fluents $?rest-parts))
  (time ?any-time) ; used to continuously attempt to request the service until success
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (ros-msgs-set-field ?new-req "pddl_instance" "rcll")
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (assert (service-request-meta (service ?s) (request-id ?id) (meta init-fluents)))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule pddl-init-fluents-response-received
" Get response, read it and delete."
  (confval (path "/pddl/manager_node") (value ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_fluents"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?) (request-id ?id) (meta init-fluents))
  ?st <- (start-task (name pddl) (state ACTIVE) (parts init-fluents $?rest-parts))
=>
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (bind ?fluents (ros-msgs-get-field ?ptr "fluents"))
    (foreach ?fluent ?fluents
      (bind ?instance (ros-msgs-get-field ?fluent "pddl_instance"))
      (bind ?name (ros-msgs-get-field ?fluent "name"))
      (bind ?args (ros-msgs-get-field ?fluent "args"))
	  (bind ?arg-syms (create$))
	  (foreach ?arg ?args
	    (bind ?arg-syms (create$ ?arg-syms (sym-cat ?arg)))
	    (assert (pddl-fluent (name (sym-cat ?name)) (params ?arg-syms) (instance ?instance)))
	  )
    )
    (modify ?st (parts $?rest-parts))
   else
    (printout error "Failed to fetch fluents \"init-fluents\":" ?error crlf)
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)

(defrule pddl-request-load-functions
  (confval (path "/pddl/manager_node") (value ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_functions"))) (type ?type))
  (not (service-request-meta (service ?s)))
  (start-task (name pddl) (state ACTIVE) (parts init-functions $?rest-parts))
  (time ?any-time) ; used to continuously attempt to request the service until success
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (ros-msgs-set-field ?new-req "pddl_instance" "rcll")
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (assert (service-request-meta (service ?s) (request-id ?id) (meta init-functions)))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule pddl-init-functions-response-received
" Get response, read it and delete."
  (confval (path "/pddl/manager_node") (value ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_functions"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?) (request-id ?id) (meta init-functions))
  ?st <- (start-task (name pddl) (state ACTIVE) (parts init-functions $?rest-parts))
=>
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (bind ?functions (ros-msgs-get-field ?ptr "functions"))
    (foreach ?function ?functions
      (bind ?instance (ros-msgs-get-field ?function "pddl_instance"))
      (bind ?name (ros-msgs-get-field ?function "name"))
      (bind ?args (ros-msgs-get-field ?function "args"))
      (bind ?value (ros-msgs-get-field ?function "value"))
      (bind ?arg-syms (create$))
      (foreach ?arg ?args
        (bind ?arg-syms (create$ ?arg-syms (sym-cat ?arg)))
      )
      (assert (pddl-numeric-fluent (name (sym-cat ?name)) (params ?arg-syms) (value ?value) (instance ?instance)))
    )
    (modify ?st (parts $?rest-parts))
   else
    (printout error "Failed to fetch fluents \"init-functions\":" ?error crlf)
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)
