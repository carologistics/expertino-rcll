{% extends "problem.jinja.pddl" %}
{% block objects %}
     wp1 wp2 - product
{% endblock %}
{% block init %}
    (step wp1 base-red)
    (next-step wp1 base-red ring-blue1)
    (next-step wp1 ring-blue1 ring-yellow2)
    (next-step wp1 ring-yellow2 ring-blue3)
    (next-step wp1 ring-blue3 cap-grey)
    (next-step wp1 cap-grey deliver)
    (next-step wp1 deliver done)

    (step wp2 base-black)
    (next-step wp2 base-black ring-green1)
    (next-step wp2 ring-green1 ring-yellow2)
    (next-step wp2 ring-yellow2 ring-orange3)
    (next-step wp2 ring-orange3 cap-black)
    (next-step wp2 cap-black deliver)
    (next-step wp2 deliver done)
    (spawnable wp1)
    (spawnable wp2)
{% endblock %}
{% block goals %}
    (step wp1 done)
    (step wp2 done)
{% endblock %}
