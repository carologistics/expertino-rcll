(deftemplate pddl-action-condition
  (slot instance (type SYMBOL))
  (slot plan (type SYMBOL))
  (slot id (type SYMBOL))
  (slot context (type SYMBOL))
  (slot instance-update (type FLOAT))
  (slot condition-type (type SYMBOL) (allowed-values ALL START OVERALL END) (default START)) ; TODO: only supports START
  (slot state (type SYMBOL) (allowed-values PENDING CHECK-CONDITION CONDITION-SAT CONDITION-UNSAT) (default PENDING))
  (multislot unsatisfied-conditions (type STRING) (default (create$)))
)
