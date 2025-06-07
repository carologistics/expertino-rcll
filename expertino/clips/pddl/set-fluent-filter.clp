(defrule pddl-set-fluent-filter
  (declare (salience ?*PRIORITY-PDDL-SET-FLUENT-FILTER*))
  ?pf <- (planning-filter (type FLUENTS) (instance ?instance) (goal ?goal) (filter $?filter))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/set_fluent_filter"))) (type ?type))
  (not (service-request-meta (service ?s)))
  (time ?any-time) ; used to continuously attempt to request the service until success
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (ros-msgs-set-field ?new-req "pddl_instance" ?instance)
  (ros-msgs-set-field ?new-req "goal_instance" ?goal)
  (ros-msgs-set-field ?new-req "fluents" ?filter)
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (assert (service-request-meta (service ?s) (request-id ?id) (meta ?instance)))
    (modify ?pi-f (busy-with SET-FLUENT-FILTER))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
  (retract ?pf)
)

(defrule pddl-set-fluent-filter-response-received
" Get response, read it and delete."
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (busy-with SET-FLUENT-FILTER))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/set_fluent_filter"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?s) (request-id ?id) (meta ?instance))
=>
  (modify ?pi-f (busy-with FALSE))
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (printout green ?success " " ?error crlf)
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)
