; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(defrule goal-expansion-expand-demo-goal
	?g <- (goal (id ?goal-id) (class DEMO-GOAL) (mode SELECTED) (parent ?parent)
	            (params target-pos ?zone robot ?robot))

	(domain-fact (name at) (param-values ?robot ?curr-loc ?curr-side))
	=>
	;(pddl-request-plan ?goal-id (str-cat "(and (at "?robot " " ?zone " INPUT))"))

	(plan-assert-sequential (sym-cat DEMO-GOAL-PLAN- (gensym*)) ?goal-id ROBOT1
		(create$ (plan-assert-action move  ?robot ?curr-loc ?curr-side ?zone INPUT)
		;(plan-assert-action wp-put ?robot ?wp ?mps INPUT (get-wp-complexity ?wp))
		)
	)
	(modify ?g (mode EXPANDED))
)
