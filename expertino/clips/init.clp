; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(defrule init-load-domain
  (not (domain-loaded))
=>
  (unwatch facts time)
  (unwatch rules time-retract)
  (bind ?share-dir (ament-index-get-package-share-directory "expertino"))
  ; (parse-pddl-domain (str-cat ?share-dir "/clips/expertino/domain.pddl"))
  (config-load (str-cat ?share-dir "/params/agent_config.yaml") "/")

  (assert (domain-loaded))
)

 (defrule init-load-initial-facts
 " Load all initial facts on startup of the game "
   (domain-loaded)
   (confval (path "/game/parameters/rcll/team_name") (value ?team-name))
   =>
   (printout info "Initializing game-state and robot facts" crlf)
   (foreach ?robot (create$ ROBOT1 ROBOT2 ROBOT3)
     (assert
       (robot (name ?robot) (number ?robot-index) (state ACTIVE)) 
     )
   )
;   (assert
;     (domain-object (name INPUT) (type mps-side))
;     (domain-object (name OUTPUT) (type mps-side))
;     (domain-object (name WAIT) (type mps-side))
;     (domain-object (name START) (type zone))
;     (domain-object (name M-Z43) (type zone))
;     (domain-object (name M-BS) (type mps))
;     (domain-object (name C-BS) (type mps))
;   )
   (assert (game-state (team ?team-name)))
;   (assert (game-time 0.))
   (assert (domain-facts-loaded))
 )
