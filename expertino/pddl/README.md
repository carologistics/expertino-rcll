# description of files

domain_base.jinja.pddl                  definition of constants, types, predicates in the planning domain
domain_planning_actions.jinja.pddl      definition of high-level action schemas for order planning
planning_domain.jinja.pddl              combines domain_base and high-level action schemas in domain_planning_actions
                                            to be used by the planner for order-level planning. Used to filter actions for planning.
domain.jinja.pddl                       combines domain_base and action schemas in domain_planning_actions AND all the sub_actions
example_problem.jinja.pddl              example problem for testing the planner (we try to re-create this during add-order-to-problem)
problem.jinja.pddl                      instantiates the base problem and adds invariant predicates. Used to instantiate a base managed_problem in the PDDL manager.
pay_from_bs.jinja.pddl                  low-level planning actions used to re-plan a pay_from_bs action
spawn_and_transport.jinja.pddl          low-level planning actions used to re-plan a spawn_and_transport action

sub_actions/base_transport.jinja.pddl               defines sub-actions for the base-transport action
sub_actions/transport_to_slide.jinja.pddl           defines sub-actions for the transport-to-slide action
sub_actions/transport.jinja.pddl                    defines sub-actions for the transport action

