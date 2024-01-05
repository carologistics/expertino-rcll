; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(defrule goal-selection-select
	?g <- (goal (id ?goal-id) (mode FORMULATED))
	(not (goal (id DEMO-GOAL) (mode ~FORMULATED)))
	=>
	(modify ?g (mode SELECTED))
)
