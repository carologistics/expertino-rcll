
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
    (parts init-cfg init-problem init-planning-actions init-replanning-actions init-fluents)
  )
)

(defrule init-from-config
  ?st <- (start-task (name pddl) (state ACTIVE) (parts init-cfg $?rest-parts))
  (confval (path "/pddl/manager_node") (value ?node))
  =>
  (assert (pddl-manager (node ?node)))
  (modify ?st (parts ?rest-parts))
) 

(defrule pddl-request-load-problem-instance
  (pddl-manager (node ?node))
  (confval (path "/pddl/problem_instance") (value ?instance))
  (confval (path "/pddl/pddl_dir") (value ?dir))
  (confval (path "/pddl/init_domain_file") (value ?domain))
  (confval (path "/pddl/init_problem_file") (value ?problem))
  (start-task (name pddl) (state ACTIVE) (parts init-problem $?rest-parts))
  =>
  (bind ?share-dir (ament-index-get-package-share-directory "expertino"))
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
  (bind ?share-dir (ament-index-get-package-share-directory "expertino"))
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
  (bind ?share-dir (ament-index-get-package-share-directory "expertino"))
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
)

(defrule pddl-init-load-facts-done
  (pddl-get-fluents (instance ?instance) (state DONE))
  (pddl-get-numeric-fluents (instance ?instance) (state DONE))
  (confval (path "/pddl/problem_instance") (value ?instance-str&:(eq ?instance (sym-cat ?instance-str))))
  ?st <- (start-task (name pddl) (state ACTIVE) (parts init-fluents $?rest-parts))
=>
  (modify ?st (parts $?rest-parts))
)

;(defrule pddl-init-plan-client
;  (confval (path "/pddl/manager_node") (value ?node))
;  (start-task (name pddl) (state ACTIVE) (parts init-planner $?rest-parts))
;  =>
;  (cx-pddl-interfaces-plan-temporal-create-client (str-cat ?node "/temp_plan"))
;)

;(defrule pddl-init-plan-client-successful
;  (confval (path "/pddl/manager_node") (value ?node))
;  (cx-pddl-interfaces-plan-temporal-client (server ?s&:(eq ?s (str-cat ?node "/temp_plan"))))
;  ?st <- (start-task (name pddl) (state ACTIVE) (parts init-planner $?rest-parts))
;  =>
;  (modify ?st (parts ?rest-parts))
;)
