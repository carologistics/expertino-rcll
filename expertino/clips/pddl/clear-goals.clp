(defrule pddl-request-cleear-goals
  (pddl-clear-goals (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  (pddl-instance (name ?instance) (state LOADED))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/clear_goals"))) (type ?type))
  (not (service-request-meta (service ?s)))
  (time ?any-time) ; used to continuously attempt to request the service until success
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (ros-msgs-set-field ?new-req "pddl_instance" ?instance)
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (assert (service-request-meta (service ?s) (request-id ?id) (meta ?instance)))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule pddl-clear-goals-response-received
" Get response, read it and delete."
  ?clear-goals-f <- (pddl-clear-goals (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  (pddl-instance (name ?instance) (state LOADED))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/clear_goals"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?s) (request-id ?id) (meta ?instance))
=>
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (modify ?clear-goals-f (state DONE))
   else
    (modify ?clear-goals-f (state ERROR) (error ?error))
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)
