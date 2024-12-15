(deftemplate planner-call
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

(defrule plan-main-problem
  (confval (path "/pddl/manager_node") (value ?node))
  (confval (path "/pddl/problem_instance") (value ?instance))
  (expertino-msgs-plan-temporal-client (server ?server&:(eq ?server (str-cat ?node "/temp_plan"))))
  (not (planner-call (status PENDING)))
  (not (test-plan-triggered))
  =>
  (bind ?goal (expertino-msgs-plan-temporal-goal-create))
  (assert (planner-call (context test-plan) (goal ?goal)))
  (expertino-msgs-plan-temporal-goal-set-field ?goal "pddl_instance" ?instance)
  (expertino-msgs-plan-temporal-send-goal ?goal ?server)
  (assert (test-plan-triggered))
)

(defrule plan-update-status
  ?gr-f <- (expertino-msgs-plan-temporal-goal-response (server ?server) (client-goal-handle-ptr ?cgh-ptr))
  ?pc-f <- (planner-call (status PENDING))
  =>
  (bind ?status (expertino-msgs-plan-temporal-client-goal-handle-get-status ?cgh-ptr))
  (bind ?uuid (expertino-msgs-plan-temporal-client-goal-handle-get-goal-id ?cgh-ptr))
  (modify ?pc-f (uuid ?uuid)
    (status (client-status-to-sym ?status))
    (client-goal-handle ?cgh-ptr)
  )
  (printout green "got goal response " (client-status-to-sym ?status) crlf)
)

(defrule plan-finish-plan
  ?pc <- (planner-call (status ~PENDING) (client-goal-handle ?cgh-ptr))
  (time ?) ; poll the update
  =>
  (bind ?status (expertino-msgs-plan-temporal-client-goal-handle-get-status ?cgh-ptr))
  (modify ?pc (status (client-status-to-sym ?status)))
)

(defrule plan-get-result
  ?pc-f <- (planner-call (client-goal-handle ?cgh-ptr) (goal ?goal-ptr) (uuid ?goal-id))
  ?wr-f <- (expertino-msgs-plan-temporal-wrapped-result (server "/pddl_manager/temp_plan") (goal-id ?goal-id) (code SUCCEEDED) (result-ptr ?res-ptr))
  =>
  (bind ?plan-found (expertino-msgs-plan-temporal-result-get-field ?res-ptr "success"))
  (if ?plan-found then
    (bind ?plan (expertino-msgs-plan-temporal-result-get-field ?res-ptr "actions"))
	(foreach ?action ?plan
	(bind ?instance (expertino-msgs-timed-plan-action-get-field ?action "pddl_instance"))
	(bind ?name (expertino-msgs-timed-plan-action-get-field ?action "name"))
	(bind ?args (expertino-msgs-timed-plan-action-get-field ?action "args"))
	(assert (pddl-action (id (gensym*)) (instance ?instance) (name ?name) (params ?args) (state INITIAL)))
	)
  )
	(expertino-msgs-plan-temporal-result-destroy ?res-ptr)
	(expertino-msgs-plan-temporal-goal-destroy ?goal-ptr)
	(expertino-msgs-plan-temporal-client-goal-handle-destroy ?cgh-ptr)
   (retract ?pc-f)
   (retract ?wr-f)
)
