; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(defrule goal-reasoner-completed
	?g <- (goal (id ?goal-id) (mode FINISHED) (outcome COMPLETED))
	=>
	(printout t "Goal '" ?goal-id "' has been completed, cleaning up" crlf)
	(delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
		(delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
			(retract ?a)
		)
		(retract ?p)
	)
	(retract ?g)
)

(defrule goal-reasoner-failed
	?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED) (retries ?num-tries))
	=>
	(printout error "Goal '" ?goal-id "' has failed, cleaning up" crlf)
	(delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
		(delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
			(retract ?a)
		)
		(retract ?p)
	)
	(bind ?num-tries (+ ?num-tries 1))
	(if (< ?num-tries ?*GOAL-MAX-TRIES*)
	then
		(printout t "Triggering re-expansion" crlf)
		(modify ?g (mode SELECTED) (retries ?num-tries))
	else
		(printout t "Goal failed " ?num-tries " times, aborting" crlf)
		(retract ?g)
	)
)
