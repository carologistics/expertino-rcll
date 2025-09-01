(defrule pddl-request-load-instance
  (declare (salience ?*PRIORITY-PDDL-INSTANCES*))
  (pddl-instance (state PENDING) (name ?instance) (domain ?domain) (problem ?problem) (directory ?dir))
  (pddl-manager (node ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/add_pddl_instance"))) (type ?type))
  (not (service-request-meta (service ?s)))
  (time ?any-time) ; used to continuously attempt to request the service until success
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (ros-msgs-set-field ?new-req "name" ?instance)
  (bind ?share-dir (ament-index-get-package-share-directory "expertino_blocksworld"))
  (ros-msgs-set-field ?new-req "directory" (str-cat ?share-dir "/" ?dir))
  (ros-msgs-set-field ?new-req "domain_file" ?domain)
  (ros-msgs-set-field ?new-req "problem_file" ?problem)
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (assert (service-request-meta (service ?s) (request-id ?id) (meta ?instance)))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule pddl-load-instance-response-received
" Get response, make sure that it succeeded and delete it afterwards."
  (pddl-manager (node ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/add_pddl_instance"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (service-request-meta (service ?) (request-id ?id) (meta ?meta))
  ?instance-f <-(pddl-instance (state PENDING) (name ?meta))
=>
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (modify ?instance-f (state LOADED))
   else
    (modify ?instance-f (state ERROR) (error ?error))
    (printout error "Failed to set problem instance \"" ?meta "\":" ?error crlf)
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)
