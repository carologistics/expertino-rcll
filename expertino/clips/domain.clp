(defrule load-domain
  (executive-init)
  (not (domain-loaded))
=>
  (parse-pddl-domain (path-resolve "domain.pddl"))
  (assert (domain-loaded))
)

(defrule domain-load-initial-facts
" Load all initial domain facts on startup of the game "
  (domain-loaded)
  =>
  (printout info "Initializing worldmodel" crlf)
  (bind ?team-color MAGENTA)
  (foreach ?robot (create$ robot1 robot2 robot3)
    (assert
      (domain-fact (name at) (param-values ?robot START INPUT))
    )
  )
  (assert (domain-facts-loaded))
)
