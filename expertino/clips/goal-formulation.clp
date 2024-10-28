; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(defrule goal-formulation-demo-goal
	(domain-loaded)
	(not (goal))
	(not (goal-already-tried))
	(domain-facts-loaded)
	(game-state (phase SETUP))
	=>
	(assert (goal (id DEMO-GOAL) (class DEMO-GOAL) (params target-pos M-Z43 robot ROBOT1)))
	; This is just to make sure we formulate the goal only once.
	; In an actual domain this would be more sophisticated.
	(assert (goal-already-tried))
)
