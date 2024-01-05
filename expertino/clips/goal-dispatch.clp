; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(defrule goal-dispatch
	?g <- (goal (mode COMMITTED))
	=>
	(modify ?g (mode DISPATCHED))
)
