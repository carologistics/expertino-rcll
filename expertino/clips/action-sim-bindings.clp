; (defrule action-sim-bindng-move
; " Override skiller invocation for actions with an action-task mapping by
;   putting them in state WAITING before the skiller is invoked.
; "
;   ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (state WAITING)
;     (action-name ?action-name) (robot ?robot-str)
;     (param-values ?robot ?from ?from-side ?mps ?mps-side)
;   )
;   (domain-object (name ?mps) (type mps))
;   (not (rcll-agent-task (robot ?robot) (task-id ?seq)))
;   (current-rcll-agent-task-id (robot ?robot&:(eq (str-cat ?robot) ?robot-str)) (task-id ?curr-task))
;   (not (rcll-agent-task (robot ?robot) (task-id ?curr-task)))
;   (action-task-executor-enable (name ?action-name))
;   =>
;   (assert (rcll-agent-task (task-id ?curr-task) (robot ?robot) (task-type Move)
;    (machine ?mps) (side ?mps-side) (goal-id ?goal-id) (plan-id ?plan-id)
;    (action-id ?id)
;   ))
; (bind ?next-task (+ ?curr-task 1))
; (assert (rcll-agent-task (task-id ?next-task) (robot ?robot) (task-type Move)
; (machine M-CS1) (side INPUT) (goal-id ?goal-id) (plan-id ?plan-id)
; (action-id ?id)
; ))
; (bind ?next-task (+ ?next-task 1))
; (assert (rcll-agent-task (task-id ?next-task) (robot ?robot) (task-type Retrieve)
; (machine M-CS1) (side LEFT) (goal-id ?goal-id) (plan-id ?plan-id)
; (action-id ?id)
; ))
; (bind ?next-task (+ ?next-task 1))
; (assert (rcll-agent-task (task-id ?next-task) (robot ?robot) (task-type Deliver)
; (machine M-CS1) (side INPUT) (goal-id ?goal-id) (plan-id ?plan-id)
; (action-id ?id)
; ))
; )
