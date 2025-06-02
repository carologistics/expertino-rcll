(defrule pddl-request-get-objects
  (pddl-get-objects (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_objects"))) (type ?type))
  (not (service-request-meta (service ?s)))
  (time ?any-time) ; used to continuously attempt to request the service until success
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (ros-msgs-set-field ?new-req "pddl_instance" ?instance)
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (modify ?pi-f (busy-with GET-OBJECTS))
    (assert (service-request-meta (service ?s) (request-id ?id) (meta ?instance)))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule pddl-get-objects-response-received
" Get response, read it and delete."
  ?get-facts-f <- (pddl-get-objects (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (busy-with GET-OBJECTS))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_objects"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?s) (request-id ?id) (meta ?instance))
=>
  (modify ?pi-f (busy-with FALSE))
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (bind ?objects (ros-msgs-get-field ?ptr "objects"))
    (foreach ?object ?objects
      (bind ?instance (sym-cat (ros-msgs-get-field ?object "pddl_instance")))
      (bind ?name (sym-cat (ros-msgs-get-field ?object "name")))
      (bind ?type (ros-msgs-get-field ?object "type"))
      (assert (pddl-object (name ?name) (type ?type) (instance ?instance)))
    )
    (modify ?get-facts-f (state DONE))
   else
    (modify ?get-facts-f (state ERROR) (error ?error))
    (printout error "Failed to get objects (" ?instance "):" ?error crlf)
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)