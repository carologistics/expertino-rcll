(defrule plan-wm-update-create-subscription
  "Create a subscription on the pddl_manager's instance update topic"
  (not (ros-msgs-subscription (topic "pddl_manager/instance_update")))
=>
  (ros-msgs-create-subscription "pddl_manager/instance_update" "std_msgs/msg/String")
  (printout info "Listening for String messages on /pddl_manager/instance_update" crlf)
)

(defrule plan-wm-update-recv
  "React to incoming messages on the pddl_manager's instance update topic"
  (ros-msgs-subscription (topic ?sub&:(eq ?sub "pddl_manager/instance_update")))
  ?msg-f <- (ros-msgs-message (topic ?sub) (msg-ptr ?inc-msg))
  =>
  (bind ?recv (ros-msgs-get-field ?inc-msg "data"))
  (delayed-do-for-all-facts ((?pi-f pddl-instance-update)) (eq ?pi-f:instance (sym-cat ?recv))
    (retract ?pi-f)
  )
  (assert (pddl-instance-update (instance (sym-cat ?recv)) (last-updated (now))))
  (ros-msgs-destroy-message ?inc-msg)
  (retract ?msg-f)
)

(defrule pddl-set-goals-done
  ?set-goals-f <- (pddl-set-goals (instance ?instance) (state DONE)) 
  ?pi-f <- (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  =>
  (do-for-all-facts ((?ppf pddl-goal-fluent)) (eq ?ppf:instance ?instance)
    (retract ?ppf) 
  ) 
  (do-for-all-facts ((?ppf pddl-goal-numeric-fluent)) (eq ?ppf:instance ?instance)
    (retract ?ppf)
  )
)
