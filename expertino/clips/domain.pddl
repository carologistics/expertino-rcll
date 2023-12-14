;****************************************************************************
;  rcll_domain_production.pddl: RoboCup Logistics League Production Model
;
;  Created: Fri Feb 24 23:20:38 2017
;  Copyright  2017  Tim Niemueller [www.niemueller.de]
;             2018  Till Hofmann
;****************************************************************************

;  This program is free software; you can redistribute it and/or modify
;  it under the terms of the GNU General Public License as published by
;  the Free Software Foundation; either version 2 of the License, or
;  (at your option) any later version.
;
;  This program is distributed in the hope that it will be useful,
;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;  GNU Library General Public License for more details.
;
;  Read the full text in the LICENSE.GPL file in the doc directory.

(define (domain rcll-production)
(:requirements :strips :typing)

(:types
	robot - object
	location - object
	mps - location
	mps-side - object
	base-color - object
	cap-color - object
	ring-color - object
	ss-operation - object
	cs-operation - object
	cs-statename - object
	order - object
	order-complexity-value - object
	workpiece - object
	cap-carrier - workpiece
	shelf-spot - object
	number - object
)

(:constants
	START - location
	INPUT OUTPUT - mps-side
	BASE_NONE BASE_CLEAR BASE_RED BASE_BLACK BASE_SILVER - base-color
	CAP_NONE CAP_BLACK CAP_GREY - cap-color
	RING_NONE RING_BLUE RING_GREEN RING_ORANGE RING_YELLOW - ring-color
	LEFT MIDDLE RIGHT - shelf-spot
)

(:predicates
	(at ?r - robot ?m - location ?side - mps-side)
	(holding ?r - robot ?wp - workpiece)
	(wp-at ?wp - workpiece ?m - mps ?side - mps-side)
	(wp-base-color ?wp - workpiece ?col - base-color)
	(wp-ring1-color ?wp - workpiece ?col - ring-color)
	(wp-ring2-color ?wp - workpiece ?col - ring-color)
	(wp-ring3-color ?wp - workpiece ?col - ring-color)
	(wp-cap-color ?wp - workpiece ?col - cap-color)
	(wp-on-shelf ?wp - workpiece ?m - mps ?spot - shelf-spot)
)

(:action move
	:parameters (?r - robot ?from - location ?from-side - mps-side
	             ?to - mps ?to-side - mps-side)
	:precondition (at ?r ?from ?from-side)
	:effect (and (not (at ?r ?from ?from-side))
	             (at ?r ?to ?to-side)
	        )
)
)
