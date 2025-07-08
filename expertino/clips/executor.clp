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

(defrule executor-create
  ?pa <- (pddl-action (id ?action-id))
  ?aa <- (agenda-action-item (plan ?plan-id) (action ?action-id) (execution-state SELECTED) (worker ?worker))
  (agenda (plan ?plan-id) (state ACTIVE))
  (not (executor (pddl-action-id ?action-id)))
  ;TODO agenda-action-item is also supposed to give an assigned worker to the action
  =>
  (assert (executor (id (sym-cat EXECUTOR-(gensym*))) (pddl-action-id ?action-id) (worker ?worker) (state INIT)))
)

(defrule executor-accepted
  (executor (id ?ex-id) (state ACCEPTED) (pddl-action-id ?action-id))
  (pddl-action (id ?action-id) (params $?action-params))
  ?aa <- (agenda-action-item (action ?action-id) (execution-state SELECTED) (worker ?worker&~AGENT))
  =>
  (modify ?aa (execution-state EXECUTING))
  (assert (pddl-action-get-effect (action ?action-id) (apply TRUE) (effect-type START)))
  ;(assert (selected-order (order 2)))
)

(defrule executor-succeeded
  (executor (id ?ex-id) (state SUCCEEDED) (pddl-action-id ?action-id))
  (pddl-action (id ?action-id) (params $?action-params))
  ?aa <- (agenda-action-item (action ?action-id) (execution-state EXECUTING))
  =>
  (modify ?aa (execution-state COMPLETED))
)

(defrule executor-failed
  (executor (id ?ex-id) (state ABORTED) (pddl-action-id ?action-id))
  (pddl-action (id ?action-id) (params $?action-params))
  ?aa <- (agenda-action-item (action ?action-id) (execution-state EXECUTING))
  =>
  ;TODO set appropriate error message
  (modify ?aa (execution-state ERROR))
)

(defrule executor-agent-worker-replan-get-effect-request
  ?ex <- (executor (pddl-action-id ?action-id) (worker AGENT) (state INIT))
  (agenda-action-item (action ?action-id) (execution-state SELECTED))
  =>
  (modify ?ex (state REQUESTED))
  (printout debug "replanning ...." crlf)
  (assert (pddl-action-get-effect (action ?action-id) (effect-type END)))
)

(defrule executor-agent-worker-replan-set-goals
  ?ex <- (executor (pddl-action-id ?action-id) (worker AGENT) (state REQUESTED))
  ?aa <- (agenda-action-item (action ?action-id) (execution-state SELECTED))
  (or (pddl-effect-fluent (action ?action-id))
      (pddl-effect-numeric-fluent (action ?action-id)))
  (confval (path "/pddl/problem_instance") (value ?instance-str))
  =>
  (bind ?instance (sym-cat ?instance-str))
  (assert (pddl-clear-goals (instance ?instance) (goal ?*GOAL-INSTANCE-REPLANNING*)))

  ;assert new goals
  (delayed-do-for-all-facts ((?effect-f pddl-effect-fluent)) (eq ?effect-f:action ?action-id)
    (assert (pddl-goal-fluent (goal ?*GOAL-INSTANCE-REPLANNING*) (instance ?effect-f:instance) (name ?effect-f:name) (params ?effect-f:params)))
    (retract ?effect-f)
  )
  (delayed-do-for-all-facts ((?effect-f pddl-effect-numeric-fluent)) (eq ?effect-f:action ?action-id)
    (assert (pddl-goal-numeric-fluent (goal ?*GOAL-INSTANCE-REPLANNING*) (instance ?effect-f:instance) (name ?effect-f:name) (params ?effect-f:params) (value ?effect-f:value)))
    (retract ?effect-f)
  )
  (modify ?ex (state ACCEPTED))
  (modify ?aa (execution-state EXECUTING))
) 

(defrule executor-set-goal-for-replanning                                                    
  ?ex <- (executor (pddl-action-id ?action-id) (worker AGENT) (state ACCEPTED))
  (or (pddl-goal-fluent (instance ?instance) (goal ?goal&:(eq ?goal ?*GOAL-INSTANCE-REPLANNING*)))          
    (pddl-goal-numeric-fluent (instance ?instance) (goal ?goal&:(eq ?goal ?*GOAL-INSTANCE-REPLANNING*))))          
  ?clear-f <- (pddl-clear-goals (instance ?instance) (state DONE) (goal ?goal&:(eq ?goal ?*GOAL-INSTANCE-REPLANNING*))) 
  =>                                                                            
  ; notify to add the goal to the domain                                        
  (assert (pddl-set-goals (instance ?instance) (goal ?*GOAL-INSTANCE-REPLANNING*)))   
  (retract ?clear-f)                                                            
)  

(defrule executor-start-replanning                               
  (executor (id ?ex-id) (pddl-action-id ?action-id) (worker AGENT) (state ACCEPTED))
  ?set-f <- (pddl-set-goals (instance ?instance) (state DONE) (goal ?goal&:(eq ?goal ?*GOAL-INSTANCE-REPLANNING*)))
  (pddl-manager (node ?node))                                                   
  (pddl-instance (name ?instance) (busy-with FALSE) (state LOADED))             
  (expertino-msgs-plan-temporal-client (server ?server&:(eq ?server (str-cat ?node "/temp_plan"))))
  =>                                                                            
  (printout green "Start re-planning" crlf)                                        
  (bind ?goal (expertino-msgs-plan-temporal-goal-create))                       
  (assert (pddl-planner-call (context ?ex-id) (goal ?goal)))                 
  (expertino-msgs-plan-temporal-goal-set-field ?goal "pddl_instance" ?instance) 
  (expertino-msgs-plan-temporal-goal-set-field ?goal "goal_instance" ?*GOAL-INSTANCE-REPLANNING*)    
  (expertino-msgs-plan-temporal-send-goal ?goal ?server)                        
  (retract ?set-f)                                                              
  ;clear existing goals for the goal-instance
  (do-for-all-facts ((?goal-fluent pddl-goal-fluent)) 
    (and (eq ?goal-fluent:instance ?instance) (eq ?goal-fluent:goal ?goal))
    (retract ?goal-fluent)
  )
  (do-for-all-facts ((?goal-fluent pddl-goal-numeric-fluent)) 
    (and (eq ?goal-fluent:instance ?instance) (eq ?goal-fluent:goal ?goal))
    (retract ?goal-fluent)
  )
)

(defrule executor-agent-worker-succeeded
  ?ex <- (executor (id ?ex-id) (pddl-action-id ?action-id) (worker AGENT) (state ACCEPTED))
  (pddl-plan (id ?plan-id) (context ?ex-id))
  (agenda (plan ?plan-id))
  (agenda-action-item (plan ?plan-id))
  (not (agenda-action-item (plan ?plan-id) (execution-state ?state&~COMPLETED)))
  =>
  (modify ?ex (state SUCCEEDED))
)
