(defrule init-load-domain
  (not (domain-loaded))
=>
  (unwatch facts time)
  (unwatch rules time-retract)
  (bind ?share-dir (ament-index-get-package-share-directory "expertino_blocksworld"))
  (config-load (str-cat ?share-dir "/params/agent_config.yaml") "/")

  (assert (domain-loaded))
)

(deffunction observe-predicates-except-on-table ()
	(do-for-all-facts ((?p pddl-predicate))
		(neq ?p:name on-table)
	(assert (rl-observable-predicate 	(name ?p:name) 
										(param-types ?p:param-types) 
										(param-names ?p:param-names)))
	)
)

(deffunction observe-all-objects ()
	(do-for-all-facts ((?o pddl-object))
		TRUE
	(assert (rl-observable-object	(name ?o:name) 
									(type ?o:type)))
	)
)

(deffunction predefine-observables ()
	(assert (rl-predefined-observable (name on-table) (params block1)))
	(assert (rl-predefined-observable (name on-table) (params block2)))
	(assert (rl-predefined-observable (name on-table) (params block3)))
)

(deffunction add-robot ()
  (assert (rl-robot (name robot1)))
)

(deffunction rl-generate-observations ()
	(do-for-all-facts ((?pf pddl-fluent))
			TRUE
		(assert (rl-observation (name ?pf:name) (param-values ?pf:params)))
	)
)

(deffunction generate-pddl-actions (?instance)
  (assert (pddl-action  (instance ?instance) (id (sym-cat pickup- (gensym*))) (plan NONE) (name pickup) (params robot1 block1)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat pickup- (gensym*))) (plan NONE) (name pickup) (params robot1 block2)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat pickup- (gensym*))) (plan NONE) (name pickup) (params robot1 block3)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat pickup- (gensym*))) (plan NONE) (name pickup) (params robot1 block4)))

  (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block1 block2)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block1 block3)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block1 block4)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block2 block1)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block2 block3)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block2 block4)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block3 block1)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block3 block2)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block3 block4)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block4 block1)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block4 block2)))
  (assert (pddl-action  (instance ?instance) (id (sym-cat stack- (gensym*))) (plan NONE) (name stack) (params robot1 block4 block3)))
)

(defrule init-load-domain-facts
  (domain-loaded)
  (not (domain-facts-loaded))
  (startup-completed)
  (confval (path "/pddl/problem_instance") (value ?instance-str))
  =>
  (assert (pddl-get-predicates (instance (sym-cat ?instance-str))))
  (assert (pddl-get-objects (instance (sym-cat ?instance-str))))
)

(defrule init-load-domain-facts-done
  (not (domain-facts-loaded))
  (pddl-get-predicates (instance ?instance) (state DONE))
  (pddl-get-objects (instance ?instance) (state DONE))
  (confval (path "/pddl/problem_instance") (value ?instance-str&:(eq ?instance (sym-cat ?instance-str))))
  =>
  (predefine-observables)
  (observe-predicates-except-on-table)
  (observe-all-objects)
  (add-robot)
  (generate-pddl-actions ?instance)
  (assert (domain-facts-loaded))
)

(defrule domain-episode-finished-success
    (declare (salience ?*SALIENCE-RL-EPISODE-END-SUCCESS*))
    (not (rl-episode-end))
    (pddl-fluent (name on) (params block2 block1))
    (pddl-fluent (name on) (params block3 block2))
    (pddl-fluent (name on) (params block4 block3))
    =>
    (assert (rl-episode-end (success TRUE)))
)   