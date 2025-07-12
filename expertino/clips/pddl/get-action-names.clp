(defrule pddl-request-get-action-names
  (declare (salience ?*PRIORITY-PDDL-GET-ACTION-NAMES*))
  (pddl-action-names (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_action_names"))) (type ?type))
  (not (service-request-meta (service ?s)))
  (time ?any-time) ; used to continuously attempt to request the service until success
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (ros-msgs-set-field ?new-req "pddl_instance" ?instance)
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (assert (service-request-meta (service ?s) (request-id ?id) (meta ?instance)))
    (modify ?pi-f (busy-with GET-ACTION-NAMES))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule pddl-get-action-names-response-received
" Get response, read it and delete."
  ?action-names-f <- (pddl-action-names (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (busy-with GET-ACTION-NAMES))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_action_names"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?s) (request-id ?id) (meta ?instance))
=>
  (modify ?pi-f (busy-with FALSE))
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (bind ?action-names-str (ros-msgs-get-field ?ptr "action_names"))
    (bind ?action-names (create$))
    (foreach ?an ?action-names-str
      (bind ?action-names (create$ ?action-names (sym-cat ?an)))
    )
    (modify ?action-names-f (state DONE) (action-names ?action-names))
   else
    (modify ?action-names-f (state ERROR) (error ?error))
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)
