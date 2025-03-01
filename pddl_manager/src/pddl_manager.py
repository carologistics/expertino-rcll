#!/bin/env python
import rclpy
import asyncio
import os
import concurrent.futures as cf
from concurrent.futures import wait, FIRST_COMPLETED
import time

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rclpy.action import ActionServer
from expertino_msgs.srv import (
    AddFluent,
    AddFluents,
    RemoveFluents,
    AddObjects,
    RemoveObjects,
    SetFunctions,
    AddPddlInstance,
    CheckActionPrecondition,
    GetActionEffects,
    GetActionNames,
    GetFluents,
    GetFunctions,
    SetGoals,
    ClearGoals,
)
from expertino_msgs.msg import Fluent as FluentMsg, FluentEffect, FunctionEffect, Function, TimedPlanAction, Action as ActionMsg
from expertino_msgs.action import PlanTemporal
from std_msgs.msg import String

from unified_planning.engines import PlanGenerationResultStatus
from unified_planning.shortcuts import (
    Problem,
    UserType,
    Object,
    Variable,
    UPState,
    BoolType,
    RealType,
)
from unified_planning.model.timing import Timepoint, TimepointKind
from unified_planning.plans import TimeTriggeredPlan, ActionInstance
from unified_planning.plans.plan import PlanKind
from unified_planning.plot import plot_time_triggered_plan
from unified_planning.engines.compilers import Grounder, GrounderHelper
from unified_planning.model import (
    Action,
    Fluent,
    FNode,
    ExpressionManager,
    UPState,
    Problem,
    MinimizeActionCosts,
    MinimizeExpressionOnFinalState,
    MaximizeExpressionOnFinalState,
    Oversubscription,
    Expression,
    Variable,
)
from unified_planning.model.effect import EffectKind
from unified_planning.model.timing import Timing, Timepoint, TimepointKind, TimeInterval
from unified_planning.io import PDDLReader, PDDLWriter
from unified_planning.environment import get_environment

from unified_planning.model.walkers import StateEvaluator

from jinja2 import Environment, FileSystemLoader
from up_nextflap import NextFLAPImpl


import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn


class PddlManagerLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__("pddl_problem_manager")
        self.my_executor = cf.ProcessPoolExecutor(max_workers=4)
        self.add_fluent_srv = None
        self.add_fluents_srv = None
        self.rm_fluents_srv = None
        self.add_objects_srv = None
        self.rm_objects_srv = None
        self.set_functions_srv = None
        self.add_pddl_instance_srv = None
        self.check_action_precondition_srv = None
        self.get_action_effects_srv = None
        self.get_action_names_srv = None
        self.get_fluents_srv = None
        self.get_functions_srv = None
        self.set_goals_srv = None
        self.clear_goals_srv = None
        self.plan_action_server = None
        self.reader = PDDLReader()
        self.problems = {}
        self.env = get_environment()
        # different CB groups for services and the action server to enable
        # responsive callbacks while actions are running
        # Services actually change the domain and are kept exclusive for now
        # Afaik it can also be made reentrant
        self.srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.action_cb_group = ReentrantCallbackGroup()
        self.fnode_manager = self.env.expression_manager
        # useful generic time-related things
        self.start_timing = Timing(delay=0, timepoint=Timepoint(TimepointKind.START))
        self.start_interval = TimeInterval(
            lower=self.start_timing,
            upper=self.start_timing,
            is_left_open=False,
            is_right_open=False,
        )
        self.end_timing = Timing(delay=0, timepoint=Timepoint(TimepointKind.END))
        self.end_interval = TimeInterval(
            lower=self.end_timing,
            upper=self.end_timing,
            is_left_open=False,
            is_right_open=False,
        )
        self.instance_update_pub = None
        self.get_logger().info("Lifecycle node created.")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring node...")
        # Create the service when the node enters the 'inactive' state
        self.add_fluent_srv = self.create_service(
            AddFluent,
            f"{self.get_name()}/add_fluent",
            self.handle_add_fluent,
            callback_group=self.srv_cb_group,
        )
        self.add_pddl_instance_srv = self.create_service(
            AddPddlInstance,
            f"{self.get_name()}/add_pddl_instance",
            self.handle_add_pddl_instance,
            callback_group=self.srv_cb_group,
        )
        self.check_action_precondition_srv = self.create_service(
            CheckActionPrecondition,
            f"{self.get_name()}/check_action_precondition",
            self.handle_check_action_precondition,
            callback_group=self.srv_cb_group,
        )
        self.get_action_effects_srv = self.create_service(
            GetActionEffects,
            f"{self.get_name()}/get_action_effects",
            self.handle_get_action_effects,
            callback_group=self.srv_cb_group,
        )
        self.get_action_names_srv = self.create_service(
            GetActionNames,
            f"{self.get_name()}/get_action_names",
            self.handle_get_action_names,
            callback_group=self.srv_cb_group,
        )
        self.get_fluents_srv = self.create_service(
            GetFluents,
            f"{self.get_name()}/get_fluents",
            self.handle_get_fluents,
            callback_group=self.srv_cb_group,
        )
        self.add_fluents_srv = self.create_service(
            AddFluents,
            f"{self.get_name()}/add_fluents",
            self.handle_add_fluents,
            callback_group=self.srv_cb_group,
        )
        self.rm_fluents_srv = self.create_service(
            RemoveFluents,
            f"{self.get_name()}/rm_fluents",
            self.handle_rm_fluents,
            callback_group=self.srv_cb_group,
        )
        self.add_objects_srv = self.create_service(
            AddObjects,
            f"{self.get_name()}/add_objects",
            self.handle_add_objects,
            callback_group=self.srv_cb_group,
        )
        self.rm_objects_srv = self.create_service(
            RemoveObjects,
            f"{self.get_name()}/rm_objects",
            self.handle_rm_objects,
            callback_group=self.srv_cb_group,
        )
        self.set_functions_srv = self.create_service(
            SetFunctions,
            f"{self.get_name()}/set_functions",
            self.handle_set_functions,
            callback_group=self.srv_cb_group,
        )
        self.get_functions_srv = self.create_service(
            GetFunctions,
            f"{self.get_name()}/get_functions",
            self.handle_get_functions,
            callback_group=self.srv_cb_group,
        )
        self.set_goals_srv = self.create_service(
            SetGoals,
            f"{self.get_name()}/set_goals",
            self.handle_set_goals,
            callback_group=self.srv_cb_group,
        )
        self.clear_goals_srv = self.create_service(
            ClearGoals,
            f"{self.get_name()}/clear_goals",
            self.handle_clear_goals,
            callback_group=self.srv_cb_group,
        )
        self.plan_action_server = ActionServer(
            self,
            PlanTemporal,
            f"{self.get_name()}/temp_plan",
            self.plan_callback,
            callback_group=self.action_cb_group,
        )
        self.instance_update_pub = self.create_publisher(
            String,
            f"{self.get_name()}/instance_update",
            10
        )
        self.get_logger().info("Service created successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating node...")
        self.get_logger().info("Node is active.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating node...")
        # You can make the service unavailable here if needed
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up resources...")
        if self.srv:
            self.srv.destroy()
            self.srv = None
        self.get_logger().info("Resources cleaned up.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down node...")
        return TransitionCallbackReturn.SUCCESS

    def try_set_object(self, obj, value):
        self.get_logger().debug(
            f"Received Object: name={obj.name}, type={obj.type}"
        )
        if obj.pddl_instance not in self.problems.keys():
            return False, "Unknown pddl instance"
        instance = self.problems[obj.pddl_instance]
        try:
            if(value):
              instance.add_object(obj.name, instance.user_type(obj.type))
            else:
              raise Exception("Deletion of objects is not supported yet")
            return True, ""
        except Exception as e:
            return False, f"error while { "adding" if value else "removing" } object: {e}"

    def try_set_fluent(self, fluent, value):
        self.get_logger().debug(
            f"Received Fluent: name={fluent.name}, args={fluent.args}"
        )
        if fluent.pddl_instance not in self.problems.keys():
            return False, "Unknown pddl instance"
        instance = self.problems[fluent.pddl_instance]
        args = []
        try:
            for arg in fluent.args:
                args.append(instance.object(arg))
            grounded_fluent = self.fnode_manager.FluentExp(
                instance.fluent(fluent.name), args
            )
            instance.set_initial_value(grounded_fluent, value)
            return True, ""
        except Exception as e:
            return False, f"error while { "adding" if value else "removing" } fluent: {e}"

    def try_set_function(self, function):
        self.get_logger().debug(
            f"Received Function: name={function.name}, args={function.args}, value={function.value}"
        )
        return self.try_set_fluent(function, function.value)

    def try_set_function_goal(self, function):
        return self.try_set_fluent_goal(function, function.value)


    def try_set_fluent_goal(self, fluent, value):
        if fluent.pddl_instance not in self.problems.keys():
            return False, "Unknown pddl instance"
        instance = self.problems[fluent.pddl_instance]
        args = []
        try:
            for arg in fluent.args:
                args.append(instance.object(arg))
            grounded_fluent = self.fnode_manager.FluentExp(
                instance.fluent(fluent.name), args
            )
            instance.add_goal(grounded_fluent)
            return True, ""
        except Exception as e:
            return False, f"error while { "adding" if value else "removing" } fluent: {e}"

    def handle_add_fluent(self, request, response):
        fluent = request.fluent
        response.success, response.error = self.try_set_fluent(fluent, True)
        msg = String()
        msg.data = request.fluent.pddl_instance
        self.instance_update_pub.publish(msg)
        return response

    def handle_add_fluents(self, request, response):
        success = True
        error = ""
        for fluent in request.fluents:
            curr_success, curr_error = self.try_set_fluent(fluent, True)
            success = success and curr_success
            error = f"{error} {curr_error}"
        response.success = success
        response.error = error

        msg = String()
        msg.data = request.fluents[0].pddl_instance
        self.instance_update_pub.publish(msg)
        return response

    def handle_set_functions(self, request, response):
        success = True
        error = ""
        for function in request.functions:
            curr_success, curr_error = self.try_set_function(function)
            success = success and curr_success
            error = f"{error} {curr_error}"
        response.success = success
        response.error = error

        msg = String()
        msg.data = request.functions[0].pddl_instance
        self.instance_update_pub.publish(msg)
        return response

    def handle_rm_fluents(self, request, response):
        success = True
        error = ""
        for fluent in request.fluents:
            curr_success, curr_error = self.try_set_fluent(fluent, False)
            success = success and curr_success
            error = f"{error} {curr_error}"
        response.success = success
        response.error = error

        msg = String()
        msg.data = request.fluents[0].pddl_instance
        self.instance_update_pub.publish(msg)

        return response

    def handle_add_objects(self, request, response):
        success = True
        error = ""
        for object in request.objects:
            curr_success, curr_error = self.try_set_object(object, True)
            success = success and curr_success
            error = f"{error} {curr_error}"
        response.success = success
        response.error = error

        msg = String()
        msg.data = request.objects[0].pddl_instance
        self.instance_update_pub.publish(msg)
        return response

    def handle_rm_objects(self, request, response):
        success = True
        error = ""
        for object in request.objects:
            curr_success, curr_error = self.try_set_object(object, False)
            success = success and curr_success
            error = f"{error} {curr_error}"
        response.success = success
        response.error = error

        msg = String()
        msg.data = request.objects[0].pddl_instance
        self.instance_update_pub.publish(msg)
        return response

    def handle_clear_goals(self, request, response):
        response.success = True
        error = ""
        if not request.pddl_instance in self.problems:
            response.success = False
            response.error = "Unknown pddl instance"
            return response
        instance = self.problems[request.pddl_instance]
        try:
            instance.clear_goals()
            return response
        except Exception as e:
            response.success = False
            response.error = f"error while clearing goals: {e}"
            self.get_logger().error(f"{response.error}")
            return response

    def handle_set_goals(self, request, response):
        success = True
        error = ""
        for fluent in request.fluents:
            if not success:
                break
            curr_success, curr_error = self.try_set_fluent_goal(fluent, True)
            success = success and curr_success
            error = f"{error} {curr_error}"

        for function in request.functions:
            if not success:
                break
            curr_success, curr_error = self.try_set_function_goal(function)
            success = success and curr_success
            error = f"{error} {curr_error}"
        response.success = success
        response.error = error
        if not response.success:
            self.get_logger().error(f"{response.error}")
        return response


    def handle_add_pddl_instance(self, request, response):
        directory = request.directory
        domain = request.domain_file
        problem = request.problem_file
        name = request.name
        problem_exists = bool(request.problem_file)
        try:
            jinja_env = Environment(loader=FileSystemLoader(searchpath=directory))
            domain_template = jinja_env.get_template(domain)
            domain_rendered = domain_template.render()
            if problem_exists:
                problem_template = jinja_env.get_template(problem)
                problem_rendered = problem_template.render()
            if name in self.problems.keys():
                self.get_logger().warn(f"Overriding problem {name}")

            if problem_exists:
                self.problems[name] = self.reader.parse_problem_string(
                    domain_rendered, problem_rendered
                )
            else:
                self.problems[name] = self.reader.parse_problem_string(domain_rendered)
            self.get_logger().info(f"Loading domain {domain} {problem}")
            response.success = True
            self.get_logger().debug(f"{self.problems[name]}")
        except Exception as e:
            response.error = f"error while adding problem: {e}"
            self.get_logger().error(response.error)
            response.success = False
        return response

    def handle_get_fluents(self, request, response):
        if request.pddl_instance not in self.problems.keys():
            response.success = False
            response.error = "Unknown pddl instance"
            return response
        instance = self.problems[request.pddl_instance]
        response.fluents = []
        for f, val in instance.initial_values.items():
            args = []
            for arg in f.args:
                args.append(f"{arg}")
            if val.is_bool_constant() and val.bool_constant_value():
                # this is a normal fluent
                fluent = FluentMsg(
                    pddl_instance=request.pddl_instance,
                    name=f.fluent().name,
                    args=args,
                )
                response.fluents.append(fluent)
        response.success = True
        return response

    def handle_get_functions(self, request, response):
        if request.pddl_instance not in self.problems.keys():
            response.success = False
            response.error = "Unknown pddl instance"
            return response
        instance = self.problems[request.pddl_instance]
        response.functions = []
        for f, val in instance.initial_values.items():
            args = []
            for arg in f.args:
                args.append(f"{arg}")
            if val.is_int_constant():
                # this is a function
                func = Function(
                    pddl_instance=request.pddl_instance,
                    name=f.fluent().name,
                    args=args,
                    value=float(val.int_constant_value())
                )
                response.functions.append(func)
            elif val.is_real_constant():
                # this is a function
                func = Function(
                    pddl_instance=request.pddl_instance,
                    name=f.fluent().name,
                    args=args,
                    value=val.real_constant_value()
                )
                response.functions.append(func)
        response.success = True
        return response

    def handle_check_action_precondition(self, request, response):
        action = request.action
        self.get_logger().debug(
            f"Received Action: name={action.name}, args={action.args}"
        )
        if action.pddl_instance not in self.problems.keys():
            response.success = False
            response.error = "Unknown pddl instance"
            return response
        instance = self.problems[action.pddl_instance]
        args = []
        try:
            # Ground action
            grounded_action = self.ground_action(action)
            # Build current state
            initial_values = instance.initial_values
            current_state = UPState(initial_values)
            # Evaluate each start condition
            se = StateEvaluator(instance)
            evaluate: Callable[[FNode], FNode] = lambda exp: se.evaluate(
                exp, current_state
            )
            unsatisfied_conditions = []
            for cond, value in grounded_action.conditions.items():
                if cond == self.start_interval:
                    for val in value:
                        evaluated_cond = evaluate(val)
                        if (
                            not evaluated_cond.is_bool_constant()
                            or not evaluated_cond.bool_constant_value()
                        ):
                            unsatisfied_conditions.append(val)
                            print("unsatisfied precondition: ", val)
            response.success = True
            if unsatisfied_conditions:
                response.sat = False
                unsat_cond_strings = []
                for unsat in unsatisfied_conditions:
                    unsat_cond_strings.append(f"{unsat}")
                response.unsatisfied_conditions = unsat_cond_strings
            else:
                response.sat = True
        except Exception as e:
            response.error = f"error while checking precondition: {e}"
            self.get_logger().error(response.error)
            response.success = False
        return response

    def ground_action(self, action):
        instance = self.problems[action.pddl_instance]
        args = []
        for arg in action.args:
            args.append(instance.object(arg))
        action_def = instance.action(action.name)
        action_instance = ActionInstance(action=action_def, params=args)
        grounder = GrounderHelper(instance, prune_actions=False)
        grounded_action = grounder.ground_action(
            action=action_def, parameters=action_instance.actual_parameters
        )
        return grounded_action

    def handle_get_action_effects(self, request, response):
        action = request.action
        self.get_logger().debug(
            f"Received Action: name={action.name}, args={action.args}"
        )
        if action.pddl_instance not in self.problems.keys():
            response.success = False
            response.error = "Unknown pddl instance"
            return response

        instance = self.problems[action.pddl_instance]
        an = instance.actions
        action_names_new = []
        for action_name in an:
            action_names_new.append(action_name.name)
        args = []
        try:
            # Ground action
            grounded_action = self.ground_action(action)

            function_effects = []
            fluent_effects = []
            for cond, value in grounded_action.effects.items():
                time_point = ""
                if cond == self.start_timing:
                    time_point = "START"
                elif cond == self.end_timing:
                    time_point = "END"
                else:
                    response.error = f"Unknown effect time point: {cond}"
                    self.get_logger().error(response.error)
                    response.success = False
                    return response
                for val in value:
                    operator = ""
                    match val.kind:
                        case EffectKind.ASSIGN:
                            operator = "="
                        case EffectKind.INCREASE:
                            operator = "+"
                        case EffectKind.DECREASE:
                            operator = "-"
                        case _:
                            response.error = f"Unknown operator kind: {val.kind}"
                            self.get_logger().error(response.error)
                            response.success = False
                            return response

                    args = []
                    for arg in val.fluent.args:
                        args.append(f"{arg}")
                    if val.value.is_bool_constant():
                        # this is a normal fluent change effect
                        fluent = FluentMsg(
                            pddl_instance=action.pddl_instance,
                            name=val.fluent.fluent().name,
                            args=args,
                        )
                        fluent_effects.append(
                            FluentEffect(
                                fluent=fluent,
                                time_point=time_point,
                                value=val.value.bool_constant_value(),
                            )
                        )
                    else:
                        function_msg = Function(
                            pddl_instance=action.pddl_instance,
                            name=val.fluent.fluent().name,
                            args=args,
                        )
                        if val.value.is_int_constant():
                            # this is a function change effect
                            function_effects.append(
                                FunctionEffect(
                                    function=function_msg,
                                    time_point=time_point,
                                    operator_type=operator,
                                    value=float(val.value.int_constant_value()),
                                )
                            )
                        elif val.value.is_real_constant():
                            # this is a function change effect
                            function_effects.append(
                                FunctionEffect(
                                    function=function_msg,
                                    time_point=time_point,
                                    operator_type=operator,
                                    value=val.value.real_constant_value(),
                                )
                            )
                        elif val.value.is_fluent_exp():
                            # this is a function change effect
                            rhs_fnode = self.problems[action.pddl_instance].initial_value(val.value)
                            if rhs_fnode.is_real_constant():
                                rhs_val = rhs_fnode.real_constant_value()
                            elif rhs_fnode.is_int_constant():
                                rhs_val = float(rhs_fnode.int_constant_value())
                            else:
                                raise Exception(f"value of {val.val} is unexpected ")
                            function_effects.append(
                                FunctionEffect(
                                    function=function_msg,
                                    time_point=time_point,
                                    operator_type=operator,
                                    value=rhs_val,
                                )
                            )
                        else:
                            response.error = f"Unknown value type: {val.value}"
                            self.get_logger().error(response.error)
                            response.success = False
                            return response

            response.success = True
            response.function_effects = function_effects
            response.fluent_effects = fluent_effects
            return response
        except Exception as e:
            response.error = f"error while getting effects: {e}"
            self.get_logger().error(response.error)
            response.success = False
            return response

    def handle_get_action_names(self, request, response):
        if request.pddl_instance not in self.problems.keys():
            response.success = False
            response.error = "Unknown pddl instance"
            return response
        instance = self.problems[request.pddl_instance]
        try:
            actions = instance.actions
            action_names = []
            for action in actions:
                action_names.append(action.name)
            response.success = True
            response.action_names = action_names
            return response
        except Exception as e:
            response.error = f"error while getting action names: {e}"
            self.get_logger().error(response.error)
            response.success = False
            return response

    def plan_callback(self, goal_handle):
      self.get_logger().info("Start planning...")
      response = PlanTemporal.Result()
      request = goal_handle.request

      if request.pddl_instance not in self.problems.keys():
          response.success = False
          return response

      problem = self.problems[request.pddl_instance].clone()
      action_filter = request.action_names
      if action_filter:
        actions = problem.actions
        problem.clear_actions()
        for act in actions:
          if act.name in  action_filter:
            problem.add_action(act)
      writer = PDDLWriter(problem)
      dom = writer.get_domain()
      prob = writer.get_problem()
      future = self.my_executor.submit(run_planner_process, self.env, dom, prob)
      result = future.result()
      response.actions = []
      if result:
        response.success = True
        self.get_logger().info("Successfully planned")

        delta_threshold = 0.1
        last_time = 0.0
        equiv_class_idx = 0
        # iterate over all actions in the plan to compute regions and generate response
        for time, act, duration in result.timed_actions:
          plan_action = TimedPlanAction()
          plan_action.pddl_instance = request.pddl_instance
          if f"{act.action.name}" in writer.nto_renamings.keys():
            plan_action.name=f"{writer.get_item_named(f"{act.action.name}").name}"
          else:
            plan_action.name = f"{act.action.name}"

          plan_action.args = []
          for arg in act.actual_parameters:
            if f"{arg}" in writer.nto_renamings.keys():
              plan_action.args.append(f"{writer.get_item_named(arg.__str__())}")
            else:
              plan_action.args.append(f"{arg}")
          plan_action.start_time = float(time)
          plan_action.duration = float(duration)
          if float(time) - last_time > delta_threshold:
            equiv_class_idx += 1
            last_time = float(time)
          plan_action.equiv_class = equiv_class_idx
          response.actions.append(plan_action)
      else:
        response.success = False
      goal_handle.succeed()
      return response

def run_planner_process(env, dom, prob):
  env = get_environment()
  env.credits_stream = None
  reader = PDDLReader()
  problem = reader.parse_problem_string(dom,prob)
  env.factory.add_engine("nextflap", __name__, "NextFLAPImpl")
  with env.factory.OneshotPlanner(name="nextflap") as planner:
    result = planner.solve(problem, timeout=60.0)  # Your expensive call
    tPlan=None
    # result = planner.solve(problem, timeout=60.0)
    if result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
        if result.plan.kind == PlanKind.TIME_TRIGGERED_PLAN:
            tPlan = result.plan.convert_to(
                PlanKind.TIME_TRIGGERED_PLAN, result.plan
            )
            plan = result.plan
    else:
        return False

    return tPlan

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        # Initialize the node
        node = PddlManagerLifecycleNode()

        # Use a multi-threaded executor
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)

        # Spin the executor
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        # Ensure proper cleanup
        if node:
            executor.remove_node(node)
            node.destroy_node()
        if rclpy.ok():
          rclpy.shutdown()

if __name__ == "__main__":
    main()
