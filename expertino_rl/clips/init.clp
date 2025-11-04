; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.


(deffunction rl-observe-predicates ()
  (do-for-all-facts ((?p pddl-predicate))
    (member$ ?p:name (create$ at free spawnable usable                                 
                              token-usable on-shelf buffered 
                              can-buffer))
    (assert (rl-observable-predicate 	(name ?p:name) 
                                      (param-types ?p:param-types) 
                                      (param-names ?p:param-names)))
  )
)

(deffunction rl-observe-functions ()
  (do-for-fact ((?to pddl-type-objects))
    (eq ?to:type ring)
    (foreach ?ring ?to:objects
      (assert (rl-predefined-observable (name price) (params ?ring ZERO)))
      (assert (rl-predefined-observable (name price) (params ?ring ONE)))
      (assert (rl-predefined-observable (name price) (params ?ring TWO)))
    )
  )
  (assert (rl-predefined-observable (name pay-count) (params rs1 ZERO)))
  (assert (rl-predefined-observable (name pay-count) (params rs1 ONE)))
  (assert (rl-predefined-observable (name pay-count) (params rs1 TWO)))
  (assert (rl-predefined-observable (name pay-count) (params rs2 ZERO)))
  (assert (rl-predefined-observable (name pay-count) (params rs2 ONE)))
  (assert (rl-predefined-observable (name pay-count) (params rs2 TWO)))
)

(deffunction rl-observe-types ()
  (do-for-all-facts ((?to pddl-type-objects))
    TRUE
	  (assert (rl-observable-type (type ?to:type) 
										            (objects ?to:objects)))
	)
)

(deffunction rl-generate-observations ()
  (do-for-all-facts ((?pf pddl-fluent))
    (member$ ?pf:name (create$ at free spawnable usable                                 
                               token-usable on-shelf buffered 
                               can-buffer))
    (assert (rl-observation (name ?pf:name) (param-values ?pf:params)))
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
                            (param-values (create$ ?pnf:params ?value))))
  )
)

(deffunction rl-add-robots ()
  (assert (rl-robot (name robot1)))
  (assert (rl-robot (name robot2)))
  (assert (rl-robot (name robot3)))
)

(defrule init-load-domain
  (not (domain-loaded))
=>
  (unwatch facts time)
  (unwatch rules time-retract)
  (bind ?share-dir (ament-index-get-package-share-directory "expertino_rl"))
  ; (parse-pddl-domain (str-cat ?share-dir "/clips/expertino/domain.pddl"))
  (config-load (str-cat ?share-dir "/params/agent_config.yaml") "/")
  (expertino-rl-interfaces-start-refbox-create-client "cx_rl_node/start_refbox")
  (expertino-rl-interfaces-stop-refbox-create-client "cx_rl_node/stop_refbox")
  (assert (domain-loaded))
)

(defrule init-load-initial-facts
 " Load all initial facts on startup of the game "
   (domain-loaded)
   (not (domain-facts-loaded))
   (startup-completed)
   (confval (path "/game/parameters/rcll/team_name") (value ?team-name))
   =>
   (printout info "Initializing game-state and worker facts" crlf)
;   (foreach ?robot (create$ ROBOT1 ROBOT2 ROBOT3)
;     (assert (worker (id ?robot) (state IDLE) (type ROBOT)))
;   )
   (assert (worker (id REFBOX) (state IDLE) (type REFBOX)))
   (assert (worker (id AGENT) (state IDLE) (type AGENT)))
   (assert (game-state (team ?team-name)))
   (assert (game-time 0.))

   (rl-observe-types)
   (rl-observe-predicates)
   (rl-observe-functions)
   (rl-add-robots)
   (assert (domain-facts-loaded))
 )


