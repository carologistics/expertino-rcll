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

{% extends "domain_base.jinja.pddl" %}

{% set templates = ['domain_planning_actions.jinja.pddl', 'spawn_and_transport.jinja.pddl', 'pay_from_bs.jinja.pddl', 'sub_actions/base_transport.jinja.pddl', 'sub_actions/transport.jinja.pddl', 'sub_actions/pay_with_carrier.jinja.pddl', 'sub_actions/transport_to_slide.jinja.pddl'] %}

{% block actions %}
{% for template in templates %}
    {% include template %}
{% endfor %}
{% endblock %}
