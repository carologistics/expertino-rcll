(defrule pending-objects-send-request
  (declare (salience ?*PRIORITY-PDDL-OBJECTS*))
  (pending-pddl-object (instance ?instance) (state PENDING))
  ?pi-f <- (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  (pddl-manager (node ?node))
  (ros-msgs-client (service ?add-s&:(eq ?add-s (str-cat ?node "/add_objects"))) (type ?add-type))
  (ros-msgs-client (service ?rm-s&:(eq ?rm-s (str-cat ?node "/rm_objects"))) (type ?rm-type))
  (not (service-request-meta (service ?add-s) (meta ?instance)))
  (not (service-request-meta (service ?rm-s) (meta ?instance)))
  =>
  (bind ?request-sent FALSE)
  (bind ?object-add-msgs (create$))
  (bind ?object-rm-msgs (create$))
  (delayed-do-for-all-facts ((?ppo pending-pddl-object)) (and (eq ?ppo:state PENDING) (eq ?ppo:instance ?instance))
    (bind ?object-msg (ros-msgs-create-message "expertino_msgs/msg/Object"))
    (ros-msgs-set-field ?object-msg "pddl_instance" ?ppo:instance)
    (ros-msgs-set-field ?object-msg "name" ?ppo:name)
    (ros-msgs-set-field ?object-msg "type" ?ppo:type)
    (if ?ppo:delete then
      (bind ?object-rm-msgs (create$ ?object-rm-msgs ?object-msg))
     else
      (bind ?object-add-msgs (create$ ?object-add-msgs ?object-msg))
    )
    (modify ?ppo (state WAITING))
  )
  (if (> (length$ ?object-add-msgs) 0) then
    (bind ?new-req (ros-msgs-create-request ?add-type))
    (ros-msgs-set-field ?new-req "objects" ?object-add-msgs)
    (bind ?add-id (ros-msgs-async-send-request ?new-req ?add-s))
    (if ?add-id then
      (bind ?request-sent TRUE)
      (assert (service-request-meta (service ?add-s) (request-id ?add-id) (meta ?instance)))
     else
      (printout error "Sending of request failed, is the service " ?add-s " running?" crlf)
    )
    (ros-msgs-destroy-message ?new-req)
    (foreach ?msg ?object-add-msgs
      (ros-msgs-destroy-message ?msg)
    )
  )
  (if (> (length$ ?object-rm-msgs) 0) then
    (bind ?new-req (ros-msgs-create-request ?rm-type))
    (ros-msgs-set-field ?new-req "objects" ?object-rm-msgs)
    (bind ?rm-id (ros-msgs-async-send-request ?new-req ?rm-s))
    (if ?rm-id then
      (bind ?request-sent TRUE)
      (assert (service-request-meta (service ?rm-s) (request-id ?rm-id) (meta ?instance)))
     else
      (printout error "Sending of request failed, is the service " ?rm-s " running?" crlf)
    )
    (ros-msgs-destroy-message ?new-req)
    (foreach ?msg ?object-add-msgs
      (ros-msgs-destroy-message ?msg)
    )
  )
  (if ?request-sent then
    (modify ?pi-f (busy-with OBJECTS))
  )
)

(defrule pending-add-objects-process-response
" Process a response to the /add_objects service and clean up the associated pending facts afterwards.
"
  (pddl-manager (node ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/add_objects"))))
  ?req-f <- (service-request-meta (service ?s) (meta ?instance) (request-id ?id))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  =>
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (printout debug "Successfully added objects" crlf)
    (delayed-do-for-all-facts ((?ppo pending-pddl-object)) (and (eq ?ppo:state WAITING) (eq ?ppo:instance ?instance) (not ?ppo:delete))
      (retract ?ppo)
    )
   else
    (delayed-do-for-all-facts ((?ppo pending-pddl-object)) (and (eq ?ppo:state WAITING) (eq ?ppo:instance ?instance) (not ?ppo:delete))
      (modify ?ppo (state ERROR) (error ?error))
    )
    (printout error "Failed to add objects \"" ?instance "\":" ?error crlf)
    ; TODO: how to deal with failed adding of objects
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-f)
)

(defrule pending-rm-objects-process-response
" Process a response to the /rm_objects service and clean up the associated pending facts afterwards.
"
  (confval (path "/pddl/manager_node") (value ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/rm_objects"))))
  ?req-f <- (service-request-meta (service ?s) (meta ?instance) (request-id ?id))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  =>
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (printout debug "Successfully added objects" crlf)
    (delayed-do-for-all-facts ((?ppo pending-pddl-object)) (and (eq ?ppo:state WAITING) (eq ?ppo:instance ?instance) ?ppo:delete)
      (retract ?ppo)
    )
    (printout debug "Successfully removed objects" crlf)
   else
    (delayed-do-for-all-facts ((?ppo pending-pddl-object)) (and (eq ?ppo:state WAITING) (eq ?ppo:instance ?instance) ?ppo:delete)
      (modify ?ppo (state ERROR) (error ?error))
    )
    (printout error "Failed to remove objects \"" ?instance "\":" ?error crlf)
    ; TODO: how to deal with failed removing of objects
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-f)
)

(defrule pddl-objects-all-requests-done
  ?pi-f <- (pddl-instance (name ?instance) (busy-with OBJECTS))
  (pddl-manager (node ?node))
  (ros-msgs-client (service ?add-s&:(eq ?add-s (str-cat ?node "/add_objects"))) (type ?add-type))
  (ros-msgs-client (service ?rm-s&:(eq ?rm-s (str-cat ?node "/rm_objects"))) (type ?rm-type))
  (not (service-request-meta (service ?add-s) (meta ?instance)))
  (not (service-request-meta (service ?rm-s) (meta ?instance)))
  =>
  (modify ?pi-f (busy-with FALSE))
)
