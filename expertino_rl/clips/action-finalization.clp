(deffunction rl-generate-observations ()
  (do-for-all-facts ((?pf pddl-fluent))
    (member$ ?pf:name (create$ at free spawnable usable                                 
                               token-usable on-shelf buffered 
                               can-buffer))
    (assert (rl-observation (name ?pf:name) (params ?pf:params)))
  )
  (do-for-all-facts ((?pnf pddl-numeric-fluent))
    TRUE
    (bind ?value UNDEFINED)
    (switch (integer ?pnf:value)
      (case 0 
        then
          (bind ?value ZERO)
      )
      (case 1
        then
          (bind ?value ONE)
      )
      (case 2
        then
          (bind ?value TWO)
      )
    )
    (assert (rl-observation (name ?pnf:name) 
                            (params (create$ ?pnf:params ?value))))
  )
)

(defrule expertino-rl-action-finished
    (agenda-action-item (action ?action-id) (execution-state COMPLETED))
    ?r <- (rl-action (id ?action-id) (is-selected TRUE) (is-finished FALSE))
    =>
    (rl-generate-observations)
    (modify ?r (is-finished TRUE))

)

(defrule expertino-rl-action-error
    (agenda-action-item (action ?action-id) (execution-state ERROR))
    ?r <- (rl-action (id ?action-id) (is-selected TRUE) (is-finished FALSE))
    =>
    (rl-generate-observations)
    (modify ?r (is-finished TRUE) (reward 0))
)

