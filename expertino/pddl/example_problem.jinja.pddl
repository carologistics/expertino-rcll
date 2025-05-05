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

{% extends "problem.jinja.pddl" %}
{% block objects %}
     wp1 wp2 wp3 - product
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

    (step wp3 base-silver)
    (next-step wp3 base-silver ring-orange1)
    (next-step wp3 ring-orange1 ring-yellow2)
    (next-step wp3 ring-yellow2 ring-blue3)
    (next-step wp3 ring-blue3 cap-black)
    (next-step wp3 cap-black deliver)
    (next-step wp3 deliver done)

	(spawnable wp1)
    (spawnable wp2)
    (spawnable wp3)

    (= (price ring-blue1) 2)
    (= (price ring-blue2) 2)
    (= (price ring-blue3) 2)
    (= (price ring-yellow1) 0)
    (= (price ring-yellow2) 0)
    (= (price ring-yellow3) 0)
    (= (price ring-orange1) 0)
    (= (price ring-orange2) 0)
    (= (price ring-orange3) 0)
    (= (price ring-green1) 1)
    (= (price ring-green2) 1)
    (= (price ring-green3) 1)
{% endblock %}
{% block goals %}
    (step wp1 done)
    ;(step wp1 base-red)
    ;(step wp1 ring-blue1)
    ;(step wp1 ring-yellow2)
    ;(step wp1 ring-blue3)
    ;(step wp1 cap-grey)
    ;(step wp1 deliver)
    ;(at grey3 rs1-slide)
    ; (buffered cs1 cap-grey)
    ; (buffered cs2 cap-black)
    ;(= (pay-count rs1) 3)
    ;(= (pay-count rs2) 3)
      ;(at wp2 rs1-input)
      ;(at pay2 rs1-slide)
      ;(can-buffer cs1 cap-grey)
      ;(can-buffer cs2 cap-black)
    (step wp2 done)
    ;(step wp3 done)
{% endblock %}
