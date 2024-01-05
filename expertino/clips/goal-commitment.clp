; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(defrule goal-commitment-commit
	?g <- (goal (mode EXPANDED))
	=>
	(modify ?g (mode COMMITTED))
)

