(defrule pending-function-send-request
  (declare (salience ?*PRIORITY-PDDL-FLUENTS*))
  (pending-pddl-numeric-fluent (instance ?instance) (state PENDING))
  (not (pending-pddl-object (instance ?instance)))
  ?pi-f <- (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  (pddl-manager (node ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/set_functions"))) (type ?type))
  (not (service-request-meta (service ?s) (meta ?instance)))
  =>
  (bind ?function-msgs (create$))
  (do-for-all-facts ((?ppf pending-pddl-numeric-fluent)) (and (eq ?ppf:state PENDING) (eq ?ppf:instance ?instance))
    (bind ?function-msg (ros-msgs-create-message "expertino_msgs/msg/Function"))
    (ros-msgs-set-field ?function-msg "pddl_instance" ?ppf:instance)
    (ros-msgs-set-field ?function-msg "name" ?ppf:name)
    (ros-msgs-set-field ?function-msg "args" ?ppf:params)
    (ros-msgs-set-field ?function-msg "value" ?ppf:value)
    (bind ?function-msgs (create$ ?function-msgs ?function-msg))
    (modify ?ppf (state WAITING))
  )
  (if (> (length$ ?function-msgs) 0) then
    (bind ?new-req (ros-msgs-create-request ?type))
    (ros-msgs-set-field ?new-req "functions" ?function-msgs)
    (bind ?id (ros-msgs-async-send-request ?new-req ?s))
    (if ?id then
      (modify ?pi-f (busy-with FLUENTS))
      (assert (service-request-meta (service ?s) (request-id ?id) (meta (sym-cat ?instance))))
     else
      (printout error "Sending of request failed, is the service " ?s " running?" crlf)
    )
    (ros-msgs-destroy-message ?new-req)
    (foreach ?msg ?function-msgs
      (ros-msgs-destroy-message ?msg)
    )
  )
)

(defrule pending-set-functions-process-response
" Process a response to the /set_functions service by removing the respective pddl-numeric-fluent facts and clean up the associated pending facts afterwards.
"
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (busy-with FLUENTS))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/add_fluents"))))
  ?req-f <- (service-request-meta (service ?s) (meta ?instance) (request-id ?id))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  =>
  (modify ?pi-f (busy-with FALSE))
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (printout debug "Successfully set functions" crlf)
    (delayed-do-for-all-facts ((?ppf pending-pddl-numeric-fluent)) (and (eq ?ppf:state PENDING) (eq ?ppf:instance ?instance))
      (if (not (do-for-fact ((?fluent pddl-numeric-fluent)) (and (eq ?fluent:name ?ppf:name) (eq ?fluent:params ?ppf:params))
        (modify ?fluent (value ?ppf:value)))) then
        (assert (pddl-numeric-fluent (name ?ppf:name) (instance ?instance) (params ?ppf:values) (value ?ppf:value)))
      )
      (retract ?ppf)
    )
   else
    (printout error "Failed to remove fluents \"" ?instance "\":" ?error crlf)
    ; TODO: how to deal with failed removing of fluents
    (delayed-do-for-all-facts ((?ppf pending-pddl-fluent)) (and ?ppf:request-sent (eq ?ppf:instance ?instance) ?ppf:delete)
      (modify ?ppf (error ?error) (state ERROR))
    )
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-f)
)
