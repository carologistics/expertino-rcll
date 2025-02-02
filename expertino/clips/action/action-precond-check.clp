(defrule action-precond-check-request
  (pddl-action (id ?action-id) (name ?name) (instance ?instance) (params $?params))
  ?check-fact <- (pddl-action-precondition (id ?action-id) (state PENDING))
  (not (pddl-action-precondition (id ?action-id) (state CHECK-PRECONDITION)))
  (confval (path "/pddl/manager_node") (value ?node))
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
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
  (ros-msgs-destroy-message ?action-msg)
  (modify ?check-fact (state CHECK-PRECONDITION))
)

(defrule action-precond-check-response
  ?action-fact <- (pddl-action (id ?action-id))
  ?check-fact <- (pddl-action-precondition (id ?action-id) (state CHECK-PRECONDITION))
  (confval (path "/pddl/manager_node") (value ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/check_action_precondition"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?s) (request-id ?id) (meta ?action-id))
  =>
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (bind ?sat (ros-msgs-get-field ?ptr "sat"))
    (if ?sat then
      (modify ?check-fact (state PRECONDITION-SAT))
     else
      (modify ?check-fact (state PRECONDITION-UNSAT))
      (bind ?unsat-conds (ros-msgs-get-field ?ptr "unsatisfied_conditions"))
      (printout debug ?action-id " precondition unsat: " crlf)
      (bind ?unsats (create$))
      (foreach ?cond ?unsat-conds
        (printout debug ?cond crlf)
        (bind ?unsats (insert$ ?unsats 1 ?cond))
      )
      (modify ?check-fact (unsatisfied-preconditions $?unsats))
    ) 
   else
    (printout error "Failed to check precondition \"" ?action-id "\":" ?error crlf)
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)

(defrule action-precond-response-no-action
  (confval (path "/pddl/manager_node") (value ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/check_action_precondition"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?s) (request-id ?id) (meta ?action-id))
  (not
    (and
      (pddl-action (id ?action-id))
      (pddl-action-precondition (id ?action-id) (state CHECK-PRECONDITION))
    )
  )
  =>
  (printout warn "Received precondition check response without belonging action " ?action-id crlf)
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)

(defrule agenda-action-sat
  (pddl-action-precondition (plan ?plan-id) (id ?action-id) (state PRECONDITION-SAT) (context ?context))
  (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  =>
  (printout yellow "Action " ?action-name "[" ?action-id "]" ?action-params " has satisfied preconditions for context " ?context crlf)
)

(defrule agenda-action-unsat
  (pddl-action-precondition (plan ?plan-id) (id ?action-id) (state PRECONDITION-UNSAT) (context ?context) (unsatisfied-preconditions $?unsats))
  (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  =>
  (printout yellow "Action " ?action-name "[" ?action-id "]" ?action-params " has unsatisfied preconditions: " ?unsats " for context " ?context crlf)
)
