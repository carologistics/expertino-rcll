(defrule pddl-action-sat
  (pddl-action-condition (plan ?plan-id) (action ?action-id) (state CONDITION-SAT) (context ?context))
  (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  =>
  (printout yellow "Action " ?action-name "[" ?action-id "]" ?action-params " has satisfied preconditions for context " ?context crlf)
)

(defrule pddl-action-unsat
  (pddl-action-condition (plan ?plan-id) (action ?action-id) (state CONDITION-UNSAT) (context ?context) (unsatisfied-conditions $?unsats))
  (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  =>
  (printout yellow "Action " ?action-name "[" ?action-id "]" ?action-params " has unsatisfied preconditions: " ?unsats " for context " ?context crlf)
)
