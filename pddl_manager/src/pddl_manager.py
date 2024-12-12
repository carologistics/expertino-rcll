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
    AddPddlInstance,
    CheckActionPrecondition,
    GetActionEffects,
    GetFluents,
    GetFunctions,
)
from expertino_msgs.msg import Fluent, FluentEffect, FunctionEffect, Function
from expertino_msgs.action import CallPddlPlanner

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


class AddFluentLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__("pddl_problem_manager")
        self.my_executor = cf.ProcessPoolExecutor(max_workers=4)
        self.add_fluent_srv = None
        self.add_pddl_instance_srv = None
        self.check_action_precondition_srv = None
        self.get_action_effects_srv = None
        self.get_fluents_srv = None
        self.get_functions_srv = None
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
        self.get_logger().info("Lifecycle node created.")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring node...")
        # Create the service when the node enters the 'inactive' state
        self.add_fluent_srv = self.create_service(
            AddFluent,
            "add_fluent",
            self.handle_add_fluent,
            callback_group=self.srv_cb_group,
        )
        self.add_pddl_instance_srv = self.create_service(
            AddPddlInstance,
            "add_pddl_instance",
            self.handle_add_pddl_instance,
            callback_group=self.srv_cb_group,
        )
        self.check_action_precondition_srv = self.create_service(
            CheckActionPrecondition,
            "check_action_precondition",
            self.handle_check_action_precondition,
            callback_group=self.srv_cb_group,
        )
        self.get_action_effects_srv = self.create_service(
            GetActionEffects,
            "get_action_effects",
            self.handle_get_action_effects,
            callback_group=self.srv_cb_group,
        )
        self.get_fluents_srv = self.create_service(
            GetFluents,
            "get_fluents",
            self.handle_get_fluents,
            callback_group=self.srv_cb_group,
        )
        self.get_functions_srv = self.create_service(
            GetFunctions,
            "get_functions",
            self.handle_get_functions,
            callback_group=self.srv_cb_group,
        )
        self.plan_action_server = ActionServer(
            self,
            CallPddlPlanner,
            "pddl_plan",
            self.plan_callback,
            callback_group=self.action_cb_group,
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

    def handle_add_fluent(self, request, response):
        fluent = request.fluent
        self.get_logger().debug(
            f"Received Fluent: name={fluent.name}, args={fluent.args}"
        )
        if fluent.pddl_instance not in self.problems.keys():
            response.success = False
            response.error = "Unknown pddl instance"
            return response
        instance = self.problems[fluent.pddl_instance]
        args = []
        try:
            for arg in fluent.args:
                args.append(instance.object(arg))
            grounded_fluent = self.fnode_manager.FluentExp(
                instance.fluent(fluent.name), args
            )
            instance.set_initial_value(grounded_fluent, True)
            response.success = True
            return response
        except Exception as e:
            response.success = False
            response.error = f"error while adding fluent: {e}"
            self.get_logger().error(response.error)
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
                fluent = Fluent(
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
                    fluent = Fluent(
                        pddl_instance=action.pddl_instance,
                        name=val.fluent.fluent().name,
                        args=args,
                    )
                    if val.value.is_bool_constant():
                        # this is a normal fluent change effect
                        fluent_effects.append(
                            FluentEffect(
                                fluent=fluent,
                                time_point=time_point,
                                value=val.value.bool_constant_value(),
                            )
                        )
                    elif val.value.is_int_constant():
                        # this is a function change effect
                        function_effects.append(
                            FunctionEffect(
                                fluent=fluent,
                                time_point=time_point,
                                operator_type=operator,
                                value=float(val.value.int_constant_value()),
                            )
                        )
                    elif val.value.is_real_constant():
                        # this is a function change effect
                        function_effects.append(
                            FunctionEffect(
                                fluent=fluent,
                                time_point=time_point,
                                operator_type=operator,
                                value=val.value.real_constant_value(),
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

    def plan_callback(self, goal_handle):
      self.get_logger().info("Start planning...")
      response = CallPddlPlanner.Result()
      request = goal_handle.request

      if request.instance not in self.problems.keys():
          response.success = False
          return response

      problem = self.problems[request.instance]
      writer = PDDLWriter(problem)
      dom = writer.get_domain()
      prob = writer.get_problem()
      future = self.my_executor.submit(run_planner_process, self.env, dom, prob)
      result = future.result()
      response.success = True
      goal_handle.succeed()
      return response


def run_planner_process(env, dom, prob):
  env = get_environment()
  reader = PDDLReader()
  problem = reader.parse_problem_string(dom,prob)
  env.factory.add_engine("nextflap", __name__, "NextFLAPImpl")
  with env.factory.OneshotPlanner(name="nextflap") as planner:
    result = planner.solve(problem, timeout=60.0)  # Your expensive call
    tPlan=None
    # result = planner.solve(problem, timeout=60.0)
    if result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
        print(f"; {planner.name} found a plan!")
        if result.plan.kind == PlanKind.TIME_TRIGGERED_PLAN:
            tPlan = result.plan.convert_to(
                PlanKind.TIME_TRIGGERED_PLAN, result.plan
            )
            plan = result.plan
    else:
        print("No plan found!")

    return tPlan
def main(args=None):
    rclpy.init(args=args)
    node = AddFluentLifecycleNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
