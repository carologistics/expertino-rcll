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
    IntType,
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
from up_nextflap import NextFLAPImpl

class ManagedGoal():
    def __init__(self, problem):
        self.problem = problem
        self.goal_fluents = []
        self.object_filters = []
        self.fluent_filters = []
        self.action_filters = []

    def set_goal_fluent(self, name, args):
        grounded_args = []
        for arg in args:
            grounded_args.append(self.problem.base_problem.object(arg))
        grounded_fluent = self.problem.fnode_manager.FluentExp(
            self.problem.base_problem.fluent(name), args
        )

        self.goal_fluents.append(grounded_fluent)

    def remove_goal_fluent(self, name, args):
        raise NotImplementedError("remove_goal_fluent is not implemented")

    def get_goal_fluents(self):
        return self.goal_fluents

    def add_object_filter(self, obj):
        self.object_filters.append(obj)

    def add_fluent_filter(self, fluent):
        self.fluent_filters.append(fluent)

    def add_action_filter(self, action):
        self.action_filters.append(action)

    def remove_goal_fluent(self, fluent):
        self.goal_fluents.remove(fluent)

    def remove_object_filter(self, obj):
        self.object_filters.remove(obj)

    def remove_fluent_filter(self, fluent):
        self.fluent_filters.remove(fluent)

    def plan(self):
        goal_problem = self.problem.filter_problem().clone()

        # add the goal fluents
        for fluent in self.goal_fluents:
            goal_problem.add_goal(fluent.get_upf_fluent(goal_problem))

        # plan for the problem
        tPlan = None
        with self.problem.env.factory.OneshotPlanner(name='nextflap') as planner:
            result = planner.solve(goal_problem, timeout=60.0)
            if result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
                if result.plan.kind == PlanKind.TIME_TRIGGERED_PLAN:
                    tPlan = result.plan.convert_to(
                        PlanKind.TIME_TRIGGERED_PLAN, result.plan
                    )
            else:
                return False

        return tPlan


class ManagedProblem():
    def __init__(self, problem):
        self.goals = {}
        self.base_problem = problem.clone()

        self.goals["base"] = ManagedGoal(self)


        self.env = get_environment()
        self.fnode_manager = self.env.expression_manager
        self.env.factory.add_engine('nextflap', __name__, 'NextFLAPImpl')

    def clone(self):
        cloned_problem = ManagedProblem(self.base_problem.clone())
        cloned_problem.objects = self.objects.copy()
        cloned_problem.fluents = self.fluents.copy()
        cloned_problem.goals = self.goals.copy()
        return cloned_problem

    def filter_problem(self, action_filter=None, object_filter=None, fluent_filters=None):
        target_problem = Problem("base")

        # add the objects based on the object filters
        objects = self.get_object_list()
        for object in objects:
            if object.name in self.object_filters:
                target_problem.add_object(object)

        # add the liftd fluents based on the fluent filters
        for fluent in self.base_problem.fluents:
            if fluent.name in self.fluent_filters:
                target_problem.add_fluent(fluent)

        # set the initial values based on the fluent filters
        for f, val in self.base_problem.initial_values.items():
            args = [f"{arg}" for arg in f.args]

            if f.fluent().name in self.fluent_filters and len([obj for obj in args if obj not in self.object_filters]) == 0:
                target_problem.set_initial_value(f.fluent(), val)

        # apply the actions based on the action filters
        for action in self.base_problem.actions:
            if action.name in self.action_filters:
                target_problem.add_action(action)

        return target_problem

    def get_object_list(self):
        return self.base_problem.all_objects

    def add_object(self, name, type):
        self.base_problem.add_object(name, self.base_problem.user_type(type))

    def remove_object(self, name):
        object_list = self.get_object_list()
        for obj in object_list:
            if obj.name == name:
                object_list.remove(obj)
                break
        self.base_problem = self.filter_problem(self.get_action_list(), object_list, self.get_fluent_list())

    def get_fluent_list(self):
        return self.base_problem.fluents

    def set_fluent(self, name, args, value):
        grounded_args = []
        for arg in args:
            grounded_args.append(self.base_problem.object(arg))
        grounded_fluent = self.fnode_manager.FluentExp(
            self.base_problem.fluent(name), args
        )
        self.base_problem.set_initial_value(grounded_fluent, value)

    def get_action_list(self):
        return self.base_problem.actions

    def add_action(self, name, args):
        raise NotImplementedError("add_action is not implemented")

    def remove_action(self, name):
        actions = self.base_problem.actions
        for action in actions:
            if action.name == name:
                actions.remove(action)
                break
        self.base_problem = self.filter_problem(actions, self.get_object_list(), self.get_fluent_list())

    def add_goal(self, goal="base"):
        self.goals["base"] = ManagedGoal(self)

    def get_goal(self, goal="base"):
        return self.goals[goal]

    def add_goal_fluent(self, name, args, goal="base"):
        self.goals[goal].set_goal_fluent(name, args)

    def remove_goal_fluent(self, name, args, goal="base"):
        self.goals[goal].remove_goal_fluent(name, args)
