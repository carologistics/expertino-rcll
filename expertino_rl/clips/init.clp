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

(deffunction rl-add-robots ()
  (assert (rl-robot (name robot1)))
  ;(assert (rl-robot (name robot2)))
  ;(assert (rl-robot (name robot3)))
)

(deffunction rl-define-actions ()
  (assert
    (rl-observable-action (name transport) (param-names order) (param-types product))
    (rl-observable-action (name transport-to-cs) (param-names order) (param-types product))
    (rl-observable-action (name pay-with-carrier) (param-names c rs) (param-types carrier ring-station))
    (rl-observable-action (name pay-with-base) (param-names p rs) (param-types payment ring-station))
    (rl-observable-action (name carrier-to-input) (param-names c cs) (param-types carrier cap-station))
    (rl-observable-action (name bs-dispense) (param-names order) (param-types product))
    (rl-observable-action (name bs-dispense-pay) (param-names token) (param-types payment))
  )
)

(defrule init-load-domain
  (not (domain-loaded))
  (not (saved-facts))
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
   (not (saved-facts))
   (not (domain-facts-loaded))
   (startup-completed)
   (confval (path "/game/parameters/rcll/team_name") (value ?team-name))
   =>
   (printout info "Initializing game-state and worker facts" crlf)
;   (foreach ?robot (create$ ROBOT1 ROBOT2 ROBOT3)
;     (assert (worker (id ?robot) (state IDLE) (type ROBOT)))
;   )
   (assert (worker (id REFBOX) (state IDLE) (type REFBOX)))
   (assert (game-state (team ?team-name)))
   (assert (game-time 0.))

   (rl-observe-types)
   (rl-observe-predicates)
   (rl-observe-functions)
   (rl-add-robots)
   (rl-define-actions)
   (assert (domain-facts-loaded))
 )

(defrule domain-loaded-save-facts
  (domain-facts-loaded)
  (not (cx-rl-node (name ?name)))
  (not (saved-facts))
  =>
  (assert (cx-rl-node (name ?*CX-RL-NODE-NAME*) (mode UNSET)))
)