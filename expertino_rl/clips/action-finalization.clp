(deffunction calculate-order-points (?wp)
  (bind ?complexity 0)
  (do-for-all-facts ((?pf pddl-fluent))
    (and (eq ?pf:name next-step)
         (eq (nth$ 1 ?pf:params) ?wp)
         (str-index ring (nth$ 3 ?pf:params)) 
    )
    (bind ?complexity (+ ?complexity 1))
  )
  (printout green "Order " ?wp "has complexity " ?complexity ".")
  (bind ?points 0)
  (switch ?complexity
    (case 0
      then
        (bind ?points 20)
    )
    (case 1
      then
        (bind ?points 30)
    )
    (case 2
      then
        (bind ?points 50)
    )
    (case 3
      then
        (bind ?points 100) 
    )    
  )
  (return ?points)
)

(defrule expertino-rl-action-finished-delivery
    (declare (salience 1))
    (not (rl-episode-end))
    (not (pddl-action-get-effect (state ~DONE)))
    (not (pddl-fluent-change (state ?s)))
    (executor (action-id ?action-id) (state SUCCEEDED))
    ?r <- (rl-action (id ?action-id) (is-selected TRUE) (is-finished FALSE))
    (pddl-action (id ?action-id) (name transport) (params ?wp ?from ds-input deliver))
    =>
    (bind ?points (calculate-order-points ?wp))
    (rl-generate-observations)
    (modify ?r (is-finished TRUE) (reward ?points))
)

(defrule expertino-rl-action-finished
    (not (rl-episode-end))
    (not (pddl-action-get-effect (state ~DONE)))
    (not (pddl-fluent-change (state ?s)))
    (executor (action-id ?action-id) (state SUCCEEDED))
    ?r <- (rl-action (id ?action-id) (is-selected TRUE) (is-finished FALSE))
    =>
    (rl-generate-observations)
    (modify ?r (is-finished TRUE))

)

(defrule expertino-rl-action-error
    (executor (action-id ?action-id) (state ABORTED))
    ?r <- (rl-action (id ?action-id) (is-selected TRUE) (is-finished FALSE))
    =>
    (rl-generate-observations)
    (modify ?r (is-finished TRUE) (reward 0))
)

