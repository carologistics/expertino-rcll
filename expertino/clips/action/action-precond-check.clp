(defrule agenda-action-sat
  (pddl-action-condition (action ?action-id) (state CONDITION-SAT))
  (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  =>
  (printout yellow "Action " ?action-name "[" ?action-id "]" ?action-params " has satisfied preconditions." crlf)
)

(defrule agenda-action-unsat
  (pddl-action-condition (action ?action-id) (state CONDITION-UNSAT) (unsatisfied-conditions $?unsats))
  (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  =>
  (printout yellow "Action " ?action-name "[" ?action-id "]" ?action-params " has unsatisfied preconditions: " ?unsats crlf)
)
