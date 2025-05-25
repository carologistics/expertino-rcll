; Copyright (C) 2024 Team Carologistics
;
; Licensed under GPLv2+ license, cf. LICENSE file in project root directory.

(defglobal
  ?*PI* = 3.141592653589
  ?*2PI* = 6.2831853
  ?*PI-HALF* = 1.5707963
  ?*GOAL-MAX-TRIES* = 2
  ?*SALIENCE-LOW* = -5000
)

(defglobal

  ?*CARRIER-TO-INPUT-SUCCESS* = 500
  ?*CARRIER-TO-INPUT-FAILED* = 501
  ?*SUBACTION-INIT-SUCCESS* = 502
  ?*ACTION-GENERIC-SUCCESS* = 503 ; Added for generic PDDL action success
  ?*SUBTASK-DRIVE-TO-SRC-SUCCESS* = 504
  ?*SUBTASK-PICK-UP-SUCCESS* = 505
  ?*SUBTASK-DRIVE-TO-DEST-SUCCESS* = 506
  ?*SUBTASK-PLACE-DOWN-SUCCESS* = 507


)