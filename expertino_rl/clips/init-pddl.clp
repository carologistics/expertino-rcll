
; Copyright (c) 2024 Carologistics
;
; Licensed under the Apache License, Version 2.0 (the "License");
; you may not use this file except in compliance with the License.
; You may obtain a copy of the License at
;
;     http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS,
; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.

; A start-task routine to initialize pddl using the pddl_manager
; Parts:
;  - init-clients: start all service clients
;  - init-problem: setup an initial pddl domain
;  - init-planning-actions: retrieve the list of actions to prepare the main planning filter
;  - init-fluents: retrieve fluents and numeric fluents from pddl domain and store them
;  - init-plan-server: create a client for the planner server
;

(deffacts pddl-task
  (start-task (name pddl)
    (wait-for)
    (parts init-cfg init-clients init-problem init-planning-actions init-replanning-actions init-fluents init-objects init-planner)
  )
)

(defrule init-from-config
  ?st <- (start-task (name pddl) (state ACTIVE) (parts init-cfg $?rest-parts))
  (confval (path "/pddl/manager_node") (value ?node))
  =>
  (assert (pddl-manager (node ?node)))
  (modify ?st (parts ?rest-parts))
)

(defrule pddl-init-pddl-manager-services
" Create publisher for ros_cx_out."
  (pddl-manager (node ?node))
  ?st <- (start-task
    (name pddl) (state ACTIVE)
    (parts init-clients $?rest-parts)
  )
  (not (executive-finalize))
=>
  ; create all clients
  (bind ?services (create$
    add_fluents AddFluents
    rm_fluents RemoveFluents
    add_objects AddObjects
    rm_objects RemoveObjects
    set_functions SetFunctions
    add_pddl_instance AddPddlInstance
    check_action_condition CheckActionCondition
    get_action_effects GetActionEffects
    get_action_names GetActionNames
    get_fluents GetFluents
    get_functions GetFunctions
    get_functions GetFunctions
    get_predicates GetPredicates
    get_type_objects GetTypeObjects
    set_goals SetGoals
    clear_goals ClearGoals
    set_action_filter SetActionFilter
    set_object_filter SetObjectFilter
    create_goal_instance CreateGoalInstance
    set_fluent_filter SetFluentFilter
  ))
  (bind ?index 1)
  (bind ?length (length$ ?services))
  (while (< ?index ?length)
     (bind ?service-name (nth$ ?index ?services))
     (bind ?service-type (nth$ (+ ?index 1) ?services))
     (ros-msgs-create-client
       (str-cat ?node "/" ?service-name)
       (str-cat "cx_pddl_interfaces/srv/" ?service-type)
     )
     (bind ?index (+ ?index 2))
  )
  (modify ?st (parts $?rest-parts))
)

(defrule pddl-request-load-problem-instance
  (pddl-manager (node ?node))
  (confval (path "/pddl/problem_instance") (value ?instance))
  (confval (path "/pddl/pddl_dir") (value ?dir))
  (confval (path "/pddl/init_domain_file") (value ?domain))
  (confval (path "/pddl/init_problem_file") (value ?problem))
  (start-task (name pddl) (state ACTIVE) (parts init-problem $?rest-parts))
  =>
  (bind ?share-dir (ament-index-get-package-share-directory "expertino_rl"))
  (assert (pddl-instance (name (sym-cat ?instance)) (domain ?domain) (problem ?problem) (directory (str-cat ?share-dir "/" ?dir)) (state PENDING)))
)

(defrule pddl-init-problem-loading-successful
" Get response, make sure that it succeeded and delete it afterwards."
  (pddl-instance (state LOADED) (name ?instance))
  (confval (path "/pddl/problem_instance") (value ?instance-str&:(eq ?instance (sym-cat ?instance-str))))
  ?st <- (start-task (name pddl) (state ACTIVE) (parts init-problem $?rest-parts))
  =>
  (modify ?st (parts $?rest-parts))
)

(defrule pddl-request-load-planning-action-domain
  (pddl-manager (node ?node))
  (confval (path "/pddl/pddl_dir") (value ?dir))
  (confval (path "/pddl/planning_domain_file") (value ?domain))
  (confval (path "/pddl/planning_instance") (value ?instance))
  (start-task (name pddl) (state ACTIVE) (parts init-planning-actions $?rest-parts))
  =>
  (bind ?share-dir (ament-index-get-package-share-directory "expertino_rl"))
  (assert (pddl-instance (name (sym-cat ?instance)) (domain (str-cat ?domain)) (problem "") (directory (str-cat ?share-dir "/" ?dir)) (state PENDING)))
)

(defrule pddl-init-problem-request-planning-action-domain
  (confval (path "/pddl/planning_instance") (value ?instance-str))
  (pddl-instance (state LOADED) (name ?instance&:(eq ?instance (sym-cat ?instance-str))))
  (not (pddl-action-names (instance ?instance)))
  (start-task (name pddl) (state ACTIVE) (parts init-planning-actions $?rest-parts))
  =>
  (assert (pddl-action-names (instance ?instance)))
)

(defrule pddl-init-problem-finish-planning-action-domain
  (confval (path "/pddl/planning_instance") (value ?instance-str))
  (confval (path "/pddl/problem_instance") (value ?problem-instance-str))
  (pddl-instance (state LOADED) (name ?instance&:(eq ?instance (sym-cat ?instance-str))))
  ?pan-f <- (pddl-action-names (instance ?instance) (state DONE) (action-names $?an))
  ?st <- (start-task (name pddl) (state ACTIVE) (parts init-planning-actions $?rest-parts))
  =>
  (assert (pddl-planning-filter (id (sym-cat ?instance-str)) (filter ?an) (instance (sym-cat ?problem-instance-str)) (goal ?*GOAL-INSTANCE-BASE*) (type ACTIONS)))
  (retract ?pan-f)
  (modify ?st (parts ?rest-parts))
)

(defrule pddl-request-load-re-planning-action-domain
  (pddl-manager (node ?node))
  (confval (path "/pddl/pddl_dir") (value ?dir))
  (confval (path "/pddl/replanning_domain_file") (value ?domain))
  (confval (path "/pddl/replanning_instance") (value ?instance))
  (confval (path "/pddl/problem_instance") (value ?problem-instance-str))
  (start-task (name pddl) (state ACTIVE) (parts init-replanning-actions $?rest-parts))
  =>
  (bind ?share-dir (ament-index-get-package-share-directory "expertino_rl"))
  (assert (pddl-instance (name (sym-cat ?instance)) (domain (str-cat ?domain)) (problem "") (directory (str-cat ?share-dir "/" ?dir)) (state PENDING)))
  (assert (pddl-create-goal-instance (instance (sym-cat ?problem-instance-str)) (goal ?*GOAL-INSTANCE-REPLANNING*)))
)

(defrule pddl-init-problem-request-re-planning-action-domain
  (confval (path "/pddl/replanning_instance") (value ?instance-str))
  (pddl-instance (state LOADED) (name ?instance&:(eq ?instance (sym-cat ?instance-str))))
  (not (pddl-action-names (instance ?instance)))
  (start-task (name pddl) (state ACTIVE) (parts init-replanning-actions $?rest-parts))
  =>
  (assert (pddl-action-names (instance ?instance)))
)

(defrule pddl-init-problem-finish-re-planning-action-domain
  (confval (path "/pddl/replanning_instance") (value ?instance-str))
  (confval (path "/pddl/problem_instance") (value ?problem-instance-str))
  (pddl-instance (state LOADED) (name ?instance&:(eq ?instance (sym-cat ?instance-str))))
  ?pan-f <- (pddl-action-names (instance ?instance) (state DONE) (action-names $?an))
  ?st <- (start-task (name pddl) (state ACTIVE) (parts init-replanning-actions $?rest-parts))
  =>
  (assert (pddl-planning-filter (id (sym-cat ?instance-str)) (filter ?an) (instance (sym-cat ?problem-instance-str)) (goal ?*GOAL-INSTANCE-REPLANNING*) (type ACTIONS)))
  (retract ?pan-f)
  (modify ?st (parts ?rest-parts))
)

(defrule pddl-init-load-facts
  (start-task (name pddl) (state ACTIVE) (parts init-fluents $?rest-parts))
  (confval (path "/pddl/problem_instance") (value ?instance-str))
  =>
  (assert (pddl-get-fluents (instance (sym-cat ?instance-str))))
  (assert (pddl-get-numeric-fluents (instance (sym-cat ?instance-str))))
  (assert (pddl-get-predicates (instance (sym-cat ?instance-str))))
)

(defrule pddl-init-load-facts-done
  (pddl-get-fluents (instance ?instance) (state DONE))
  (pddl-get-numeric-fluents (instance ?instance) (state DONE))
  (pddl-get-predicates (instance ?instance) (state DONE))
  (confval (path "/pddl/problem_instance") (value ?instance-str&:(eq ?instance (sym-cat ?instance-str))))
  ?st <- (start-task (name pddl) (state ACTIVE) (parts init-fluents $?rest-parts))
=>
  (modify ?st (parts $?rest-parts))
)

(deffunction get-objects-for-all-used-types (?instance)
  (bind ?found-types (create$))
	(do-for-all-facts ((?p pddl-predicate))
		TRUE
	  (foreach ?type ?p:param-types
      (if (not (member$ ?type ?found-types)) then
        (assert (pddl-get-type-objects (instance ?instance) (type ?type)))
        (bind ?found-types (insert$ ?found-types 1 ?type))
      ) 
	  )
  )
  (assert (pddl-get-type-objects (instance ?instance) (type ring)))
)

(defrule pddl-init-load-objects
  (start-task (name pddl) (state ACTIVE) (parts init-objects $?rest-parts))
  (confval (path "/pddl/problem_instance") (value ?instance-str))
  =>
  (get-objects-for-all-used-types (sym-cat ?instance-str))
)

(defrule pddl-init-load-objects-done
  (pddl-get-type-objects (instance ?instance) (state DONE))
  (not (pddl-get-type-objects (instance ?instance) (state ?state&~DONE)))
  ?int <- (pddl-type-objects (type interactable) (objects $?int-objects))
  ?wp <- (pddl-type-objects (type workpiece) (objects $?wp-objects))
  ?prod <- (pddl-type-objects (type product) (objects $?prod-objects))
  (confval (path "/pddl/problem_instance") (value ?instance-str&:(eq ?instance (sym-cat ?instance-str))))
  ?st <- (start-task (name pddl) (state ACTIVE) (parts init-objects $?rest-parts))
  =>
  (bind ?orders (create$ o1 o2 o3 o4 o5 o6 o7 o8 o9 o10))
  (modify ?int (objects (create$ $?int-objects ?orders)))
  (modify ?wp (objects (create$ $?wp-objects ?orders)))
  (modify ?prod (objects (create$ $?prod-objects ?orders)))

  (do-for-all-facts ((?t pddl-get-type-objects)) TRUE (retract ?t))

  (modify ?st (parts $?rest-parts))
)

(defrule pddl-init-plan-client
  (confval (path "/pddl/manager_node") (value ?node))
  (start-task (name pddl) (state ACTIVE) (parts init-planner $?rest-parts))
  =>
  (cx-pddl-interfaces-plan-temporal-create-client (str-cat ?node "/temp_plan"))
)

(defrule pddl-init-plan-client-successful
  (confval (path "/pddl/manager_node") (value ?node))
  (cx-pddl-interfaces-plan-temporal-client (server ?s&:(eq ?s (str-cat ?node "/temp_plan"))))
  ?st <- (start-task (name pddl) (state ACTIVE) (parts init-planner $?rest-parts))
  =>
  (modify ?st (parts ?rest-parts))
)
