(defrule action-precond-check-request
  (declare (salience ?*PRIORITY-PDDL-CHECK-PRECONDITION*))
  (pddl-action (id ?action-id) (name ?name) (instance ?instance) (params $?params) (state CHECK-PRECONDITION))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/check_action_precondition"))) (type ?type))
  (not (service-request-meta (service ?s) (meta ?action-id)))
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (bind ?action-msg (ros-msgs-create-message "expertino_msgs/msg/Fluent"))
  (ros-msgs-set-field ?action-msg "pddl_instance" ?instance)
  (ros-msgs-set-field ?action-msg "name" ?name)
  (ros-msgs-set-field ?action-msg "args" ?params)
  (ros-msgs-set-field ?new-req "action" ?action-msg)
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (assert (service-request-meta (service ?s) (request-id ?id) (meta ?action-id)))
    (modify ?pi-f (busy-with CHECK-CONDITIONS))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
  (ros-msgs-destroy-message ?action-msg)
)

(defrule action-precond-check-response
  ?act-f <- (pddl-action (id ?action-id) (state CHECK-PRECONDITION))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (busy-with ACTION-EFFECTS))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/check_action_precondition"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?s) (request-id ?id) (meta ?action-id))
  =>
  (modify ?pi-f (busy-with FALSE))
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (bind ?sat (ros-msgs-get-field ?ptr "sat"))
    (if ?sat then
      (modify ?act-f (state PRECONDITION-SAT))
     else
      (modify ?act-f (state PRECONDITION-UNSAT))
      (bind ?unsat-conds (ros-msgs-get-field ?ptr "unsatisfied_conditions"))
      (printout debug ?action-id " precondition unsat: " crlf)
      (foreach ?cond ?unsat-conds
        (printout debug ?cond crlf)
      )
    ) 
   else
    (printout error "Failed to check precondition \"" ?action-id "\":" ?error crlf)
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)

(defrule action-precond-response-no-action
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (busy-with ACTION-EFFECTS))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/check_action_precondition"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?s) (request-id ?id) (meta ?action-id))
  (not (pddl-action (id ?action-id) (state CHECK-PRECONDITION)))
  =>
  (modify ?pi-f (busy-with FALSE))
  (printout warn "Received precondition check response without belonging action " ?action-id crlf)
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)
