
; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(define (domain rcll-production)
(:requirements :strips :typing)

(:types
	robot - object
	location - object
	mps - location
	zone - location
	mps-side - object
)

(:predicates
	(at ?r - robot ?m - location ?side - mps-side)
)

(:action move
	:parameters (?r - robot ?from - location ?from-side - mps-side
	             ?to - location ?to-side - mps-side)
	:precondition (at ?r ?from ?from-side)
	:effect (and (not (at ?r ?from ?from-side))
	             (at ?r ?to ?to-side)
	        )
)
)
