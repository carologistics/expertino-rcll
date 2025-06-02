(defrule pddl-request-get-predicates
  (pddl-get-predicates (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_predicates"))) (type ?type))
  (not (service-request-meta (service ?s)))
  (time ?any-time) ; used to continuously attempt to request the service until success
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (ros-msgs-set-field ?new-req "pddl_instance" ?instance)
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (modify ?pi-f (busy-with GET-PREDICATES))
    (assert (service-request-meta (service ?s) (request-id ?id) (meta ?instance)))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule pddl-get-predicates-response-received
" Get response, read it and delete."
  ?get-facts-f <- (pddl-get-predicates (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (busy-with GET-PREDICATES))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_predicates"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?s) (request-id ?id) (meta ?instance))
=>
  (modify ?pi-f (busy-with FALSE))
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (bind ?predicates (ros-msgs-get-field ?ptr "predicates"))
    (foreach ?predicate ?predicates
      (bind ?instance (sym-cat (ros-msgs-get-field ?predicate "pddl_instance")))
      (bind ?name (sym-cat (ros-msgs-get-field ?predicate "name")))
      (bind ?param-types (ros-msgs-get-field ?predicate "param-types"))
      (bind ?type-syms (create$))
      (foreach ?type ?param-types
        (bind ?type-syms (create$ ?type-syms (sym-cat ?type)))
      )
      (assert (pddl-predicate (name ?name) (param-types ?type-syms) (instance ?instance)))
    )
    (modify ?get-facts-f (state DONE))
   else
    (modify ?get-facts-f (state ERROR) (error ?error))
    (printout error "Failed to get predicates (" ?instance "):" ?error crlf)
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)