; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(defrule init-load-domain
  (executive-initialized)
  (not (domain-loaded))
=>
  (parse-pddl-domain (path-resolve "domain.pddl"))
  (assert (domain-loaded))
)

(defrule init-load-initial-facts
" Load all initial domain facts on startup of the game "
  (domain-loaded)
  =>
  (printout info "Initializing worldmodel" crlf)
  (bind ?team-color MAGENTA)
  (foreach ?robot (create$ ROBOT1 ROBOT2 ROBOT3)
    (assert
      (domain-fact (name at) (param-values ?robot START INPUT))
      (domain-object (name ?robot) (type robot))
    )
  )
  (assert
    (domain-object (name INPUT) (type mps-side))
    (domain-object (name OUTPUT) (type mps-side))
    (domain-object (name WAIT) (type mps-side))
    (domain-object (name START) (type mps))
    (domain-object (name MZ43) (type mps))
  )
  (assert (game-state (team "Carologistics")))
  (assert (game-time 0.))
  (assert (domain-facts-loaded))
)
