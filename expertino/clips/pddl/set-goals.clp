(defrule pddl-request-set-goals
  (declare (salience ?*PRIORITY-PDDL-SET-GOALS*))
  (pddl-set-goals (instance ?instance) (goal ?goal) (state PENDING))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/set_goals"))) (type ?type))
  (not (service-request-meta (service ?s)))
  (time ?any-time) ; used to continuously attempt to request the service until success
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (bind ?fluent-goal-msgs (create$))
  (bind ?numeric-fluent-goal-msgs (create$))
  (do-for-all-facts ((?ppf pddl-goal-fluent)) (eq ?ppf:instance ?instance)
    (bind ?fluent-msg (ros-msgs-create-message "expertino_msgs/msg/Fluent"))
    (ros-msgs-set-field ?fluent-msg "pddl_instance" ?ppf:instance)
    (ros-msgs-set-field ?fluent-msg "name" ?ppf:name)
    (ros-msgs-set-field ?fluent-msg "args" ?ppf:params)
    (bind ?fluent-goal-msgs (create$ ?fluent-goal-msgs ?fluent-msg))
  )
  (do-for-all-facts ((?ppf pddl-goal-numeric-fluent)) (eq ?ppf:instance ?instance)
    (bind ?function-msg (ros-msgs-create-message "expertino_msgs/msg/Function"))
    (ros-msgs-set-field ?function-msg "pddl_instance" ?ppf:instance)
    (ros-msgs-set-field ?function-msg "name" ?ppf:name)
    (ros-msgs-set-field ?function-msg "args" ?ppf:params)
    (ros-msgs-set-field ?function-msg "value" ?ppf:value)
    (bind ?numeric-fluent-goal-msgs (create$ ?numeric-fluent-goal-msgs ?function-msg))
  )

  (ros-msgs-set-field ?new-req "fluents" ?fluent-goal-msgs)
  (ros-msgs-set-field ?new-req "functions" ?numeric-fluent-goal-msgs)
  (ros-msgs-set-field ?new-req "goal_instance" ?goal)
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (modify ?pi-f (busy-with SET-GOALS))
    (assert (service-request-meta (service ?s) (request-id ?id) (meta ?instance)))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (foreach ?msg ?fluent-goal-msgs
    (ros-msgs-destroy-message ?msg)
  )
  (foreach ?msg ?numeric-fluent-goal-msgs
    (ros-msgs-destroy-message ?msg)
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule pddl-set-goals-response-received
" Get response, read it and delete."
  ?set-goals-f <- (pddl-set-goals (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (state LOADED) (busy-with SET-GOALS))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/set_goals"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?s) (request-id ?id) (meta ?instance))
=>
  (modify ?pi-f (busy-with FALSE))
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (modify ?set-goals-f (state DONE))
   else
    (modify ?set-goals-f (state ERROR) (error ?error))
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)
