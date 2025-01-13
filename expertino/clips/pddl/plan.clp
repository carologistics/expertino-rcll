(deftemplate pddl-planner-call
  (slot context (type SYMBOL))
  (slot uuid (type STRING))
  (slot status (type SYMBOL) (allowed-values PENDING UNKNOWN ACCEPTED EXECUTING CANCELING SUCCEEDED CANCELED ABORTED) (default PENDING))
  (slot client-goal-handle (type EXTERNAL-ADDRESS))
  (slot goal (type EXTERNAL-ADDRESS))
)

(deffunction client-status-to-sym (?status-int)
  (if (= ?status-int 0) then (return UNKNOWN))
  (if (= ?status-int 1) then (return ACCEPTED))
  (if (= ?status-int 2) then (return EXECUTING))
  (if (= ?status-int 3) then (return CANCELING))
  (if (= ?status-int 4) then (return SUCCEEDED))
  (if (= ?status-int 5) then (return CANCELED))
  (if (= ?status-int 6) then (return ABORTED))
  (printout error "Unknown client status " ?status-int " (expected [0,6])" crlf)
  (return UNKNOWN)
)

(defrule plan-start-status-checks
  ?gr-f <- (expertino-msgs-plan-temporal-goal-response (server ?server) (client-goal-handle-ptr ?cgh-ptr))
  ?pc-f <- (pddl-planner-call (status PENDING))
  =>
  (bind ?status (expertino-msgs-plan-temporal-client-goal-handle-get-status ?cgh-ptr))
  (bind ?uuid (expertino-msgs-plan-temporal-client-goal-handle-get-goal-id ?cgh-ptr))
  (modify ?pc-f (uuid ?uuid)
    (status (client-status-to-sym ?status))
    (client-goal-handle ?cgh-ptr)
  )
)

(defrule plan-update-plan-status
  ?pc <- (pddl-planner-call (status ?status&: (member$ ?status (create$ UNKNOWN ACCEPTED EXECUTING CANCELING))) (client-goal-handle ?cgh-ptr))
  (time ?) ; poll the update
  =>
  (bind ?new-status (expertino-msgs-plan-temporal-client-goal-handle-get-status ?cgh-ptr))
  (bind ?new-status (client-status-to-sym ?new-status))
  (if (neq ?status ?new-status) then
    (modify ?pc (status ?new-status))
  )
)

(defrule plan-get-result
  ?pc-f <- (pddl-planner-call (client-goal-handle ?cgh-ptr) (goal ?goal-ptr) (uuid ?goal-id))
  ?wr-f <- (expertino-msgs-plan-temporal-wrapped-result (server "/pddl_manager/temp_plan") (goal-id ?goal-id) (code SUCCEEDED) (result-ptr ?res-ptr))
  =>
  (bind ?plan-found (expertino-msgs-plan-temporal-result-get-field ?res-ptr "success"))
  (printout green "planning done" crlf)
  (bind ?plan-id (gensym*))
  (bind ?instance nil)
  (if ?plan-found then
    (bind ?plan (expertino-msgs-plan-temporal-result-get-field ?res-ptr "actions"))
    (foreach ?action ?plan
      (bind ?instance (sym-cat (expertino-msgs-timed-plan-action-get-field ?action "pddl_instance")))
      (bind ?name (sym-cat (expertino-msgs-timed-plan-action-get-field ?action "name")))
      (bind ?args (expertino-msgs-timed-plan-action-get-field ?action "args"))
      (bind ?arg-syms (create$))
      (foreach ?arg ?args
        (bind ?arg-syms (create$ ?arg-syms (sym-cat ?arg)))
      )
      (bind ?equiv_class (expertino-msgs-timed-plan-action-get-field ?action "equiv_class"))
      (bind ?ps-time (expertino-msgs-timed-plan-action-get-field ?action "start_time"))
      (bind ?p-duration (expertino-msgs-timed-plan-action-get-field ?action "duration"))
      (assert (pddl-action (id (gensym*)) (plan ?plan-id) (instance ?instance) (name ?name) (params ?args) (plan-order-class ?equiv_class) (planned-start-time ?ps-time) (planned-duration ?p-duration)))
    )
  )

  (assert (pddl-plan (id ?plan-id)))
	(expertino-msgs-plan-temporal-result-destroy ?res-ptr)
	(expertino-msgs-plan-temporal-goal-destroy ?goal-ptr)
	(expertino-msgs-plan-temporal-client-goal-handle-destroy ?cgh-ptr)
   (retract ?pc-f)
   (retract ?wr-f)
)
