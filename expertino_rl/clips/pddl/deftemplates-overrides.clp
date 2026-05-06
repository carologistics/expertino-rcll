(deftemplate pddl-action-condition
  (slot instance (type SYMBOL))
  (slot plan (type SYMBOL))
  (slot action (type SYMBOL))
  (slot context (type SYMBOL))
  (slot instance-update (type FLOAT))
  (slot condition-type (type SYMBOL) (allowed-values ALL START OVERALL END) (default START)) ; TODO: only supports START
  (slot state (type SYMBOL) (allowed-values PENDING CHECK-CONDITION CONDITION-SAT CONDITION-UNSAT) (default PENDING))
  (multislot unsatisfied-conditions (type STRING) (default (create$)))
)

(deftemplate pddl-plan
"Assert a fact of this fact to plan"
  (slot instance (type SYMBOL))
  (slot id (type SYMBOL))
  (slot goal (type SYMBOL))
  (slot goal-ptr (type EXTERNAL-ADDRESS))
  (slot plan-type (type SYMBOL) (allowed-values CLASSICAL TEMPORAL) (default CLASSICAL))
  (slot goal-handle (type EXTERNAL-ADDRESS))
  (slot output-dir (type STRING))
  (slot type (type SYMBOL) (allowed-values TEMPORAL CLASSICAL))
  (slot state (type SYMBOL) (allowed-values PENDING WAITING PLANNING REQUEST-CANCELING CANCELING CANCELED SUCCESS FAILURE) (default PENDING))
  (slot context (type SYMBOL))
)