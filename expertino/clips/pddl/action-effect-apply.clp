(deffunction pddl-apply-op (?op ?value ?new-value)
  (if (eq ?op "=") then (return ?new-value))
  (if (eq ?op "+") then (return (+ ?value ?new-value)))
  (if (eq ?op "-") then (return (- ?value ?new-value)))
  (printout error "Operator " ?op " does not match expected [=,+,-]" crlf)
  (return ?new-value)
)

(defrule action-get-action-effect-request
  (declare (salience ?*PRIORITY-PDDL-APPLY-EFFECT*))
  (pddl-manager (node ?node))
  ?apply-effect-f <- (pddl-action-apply-effect (instance ?instance) (name ?name)
    (params $?params) (state PENDING))
  ?pi-f <- (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_action_effects"))) (type ?type))
  (not (service-request-meta (service ?s) (meta ?instance)))
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (bind ?action-msg (ros-msgs-create-message "expertino_msgs/msg/Action"))
  (ros-msgs-set-field ?action-msg "pddl_instance" ?instance)
  (ros-msgs-set-field ?action-msg "name" ?name)
  (ros-msgs-set-field ?action-msg "args" ?params)
  (ros-msgs-set-field ?new-req "action" ?action-msg)
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (assert (service-request-meta (service ?s) (request-id ?id) (meta ?instance)))
    (modify ?apply-effect-f (state WAITING))
    (modify ?pi-f (busy-with ACTION-EFFECTS))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
  (ros-msgs-destroy-message ?action-msg)
)

(defrule action-apply-remaining-effect
  (declare (salience ?*PRIORITY-PDDL-APPLY-EFFECT*))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (busy-with FALSE))
  ?apply-effect-f <- (pddl-action-apply-effect (instance ?instance) (name ?name)
    (params $?params) (state START-EFFECT-APPLIED))
  =>
  (modify ?pi-f (busy-with ACTION-EFFECTS))
)

(defrule action-get-action-effect
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (busy-with ACTION-EFFECTS))
  ?apply-effect-f <- (pddl-action-apply-effect (instance ?instance) (name ?name)
    (params $?params) (effect-type ?eff-type)
    (state ?state&:(member$ ?state (create$ WAITING START-EFFECT-APPLIED))))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_action_effects"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?s) (request-id ?id) (meta ?action-id))
  =>
  (modify ?pi-f (busy-with FALSE))
  (bind ?target-time-point FALSE)
  (bind ?next-state DONE)
  (if (and (member$ ?eff-type (create$ ALL START)) (eq ?state WAITING)) then
    (bind ?target-time-point START)
    (bind ?next-state START-EFFECT-APPLIED)
   else
    (if (member$ ?eff-type (create$ ALL END)) then
      (bind ?target-time-point END)
    )
  )
  (modify ?apply-effect-f (state ?next-state))
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (bind ?fun-effs (ros-msgs-get-field ?ptr "function_effects"))
    (foreach ?fun ?fun-effs
      (bind ?function-msg (ros-msgs-get-field ?fun "function"))
      (bind ?time-point (sym-cat (ros-msgs-get-field ?fun "time_point")))
      (if (eq ?time-point ?target-time-point) then
        (bind ?op (ros-msgs-get-field ?fun "operator_type"))
        (bind ?instance (sym-cat (ros-msgs-get-field ?function-msg "pddl_instance")))
        (bind ?name (sym-cat (ros-msgs-get-field ?function-msg "name")))
        (bind ?args (ros-msgs-get-field ?function-msg "args"))
        (bind ?arg-syms (create$))
        (foreach ?arg ?args
          (bind ?arg-syms (create$ ?arg-syms (sym-cat ?arg)))
        )
        (bind ?value (ros-msgs-get-field ?fun "value"))
        (if (not (do-for-fact ((?pf pddl-numeric-fluent)) (and (eq ?name ?pf:name) (eq ?pf:params ?arg-syms))
          (assert (pending-pddl-numeric-fluent (name ?name) (params ?arg-syms)
            (value (pddl-apply-op ?op ?value ?pf:value)) (state PENDING) (instance ?instance)))
        )) then
          (printout error "pddl-numeric-fluent from action effect unknown, init to 0" crlf)
          (assert (pddl-numeric-fluent (instance ?instance) (name ?name) (params ?arg-syms) (value 0.0)))
          (assert (pending-pddl-numeric-fluent (name ?name) (params ?arg-syms)
            (value (pddl-apply-op ?op 0.0 ?value)) (state PENDING) (instance ?instance)))
        ; TODO: pending numeric effects need the operator
        ; TODO: they also dont support values dependent on other functions
        )
      )
      (ros-msgs-destroy-message ?function-msg)
    )
    (bind ?fluent-effs (ros-msgs-get-field ?ptr "fluent_effects"))
    (foreach ?fluent ?fluent-effs
      (bind ?fluent-msg (ros-msgs-get-field ?fluent "fluent"))
      (bind ?time-point (sym-cat (ros-msgs-get-field ?fluent "time_point")))
      (if (eq ?time-point ?target-time-point) then
        (bind ?instance (sym-cat (ros-msgs-get-field ?fluent-msg "pddl_instance")))
        (bind ?name (sym-cat (ros-msgs-get-field ?fluent-msg "name")))
        (bind ?args (ros-msgs-get-field ?fluent-msg "args"))
        (bind ?arg-syms (create$))
        (foreach ?arg ?args
          (bind ?arg-syms (create$ ?arg-syms (sym-cat ?arg)))
        )
        (bind ?time-point (sym-cat (ros-msgs-get-field ?fluent "time_point")))
        (bind ?value (ros-msgs-get-field ?fluent "value"))
        (assert (pending-pddl-fluent (name ?name) (params ?arg-syms)
          (delete (not ?value)) (state PENDING) (instance ?instance)))
      ); endif target-time-point matches effect
      (ros-msgs-destroy-message ?fluent-msg)
    ) 
   else
    (printout error "Failed to retrieve precondition \"" ?action-id "\":" ?error crlf)
  )
  (if (eq ?next-state DONE) then
    (ros-msgs-destroy-message ?ptr)
    (retract ?msg-f)
    (retract ?req-meta)
  )
)
