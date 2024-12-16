(defrule pddl-request-get-fluents
  (pddl-get-fluents (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  (pddl-instance (name ?instance) (state LOADED))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_fluents"))) (type ?type))
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

(defrule pddl-get-fluents-response-received
" Get response, read it and delete."
  ?get-facts-f <- (pddl-get-fluents (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  (pddl-instance (name ?instance) (state LOADED))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_fluents"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?s) (request-id ?id) (meta ?instance))
=>
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (bind ?fluents (ros-msgs-get-field ?ptr "fluents"))
    (foreach ?fluent ?fluents
      (bind ?instance (sym-cat (ros-msgs-get-field ?fluent "pddl_instance")))
      (bind ?name (sym-cat (ros-msgs-get-field ?fluent "name")))
      (bind ?args (ros-msgs-get-field ?fluent "args"))
      (bind ?arg-syms (create$))
      (foreach ?arg ?args
        (bind ?arg-syms (create$ ?arg-syms (sym-cat ?arg)))
        (assert (pddl-fluent (name ?name) (params ?arg-syms) (instance ?instance)))
      )
    )
    (modify ?get-facts-f (state DONE))
   else
    (modify ?get-facts-f (state ERROR) (error ?error))
    (printout error "Failed to get fluents (" ?instance "):" ?error crlf)
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)

(defrule pddl-request-get-functions
  (pddl-get-numeric-fluents (instance ?instance) (state PENDING))
  (pddl-instance (name ?instance) (state LOADED))
  (pddl-manager (node ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_functions"))) (type ?type))
  (not (service-request-meta (service ?s)))
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (ros-msgs-set-field ?new-req "pddl_instance" "rcll")
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (assert (service-request-meta (service ?s) (request-id ?id) (meta ?instance)))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule pddl-get-functions-response-received
" Get response, read it and delete."
  ?get-facts-f <- (pddl-get-numeric-fluents (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  (pddl-instance (name ?instance) (state LOADED))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_functions"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?) (request-id ?id) (meta ?instance))
=>
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (bind ?functions (ros-msgs-get-field ?ptr "functions"))
    (foreach ?function ?functions
      (bind ?instance (sym-cat (ros-msgs-get-field ?function "pddl_instance")))
      (bind ?name (sym-cat (ros-msgs-get-field ?function "name")))
      (bind ?args (ros-msgs-get-field ?function "args"))
      (bind ?value (ros-msgs-get-field ?function "value"))
      (bind ?arg-syms (create$))
      (foreach ?arg ?args
        (bind ?arg-syms (create$ ?arg-syms (sym-cat ?arg)))
      )
      (assert (pddl-numeric-fluent (name ?name) (params ?arg-syms)
       (value ?value) (instance ?instance)))
    )
    (modify ?get-facts-f (state DONE))
   else
    (modify ?get-facts-f (state ERROR) (error ?error))
    (printout error "Failed to get numeric fluents (" ?instance "):" ?error crlf)
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)

