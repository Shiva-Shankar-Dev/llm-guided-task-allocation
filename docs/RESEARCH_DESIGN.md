# Research basis and project design

## Objective

Convert simple natural-language instructions into validated structured robot
jobs, allocate them by reachable map distance and queued work, and recover from
stalled execution without additional LLM reasoning calls. Navigation and
obstacle avoidance are supporting execution components.

## Base papers

### SMART-LLM

Kannan, Venkatesh, and Min, *SMART-LLM: Smart Multi-Agent Robot Task Planning using
Large Language Models*, arXiv:2309.10062v2.

The method describes LLM-guided decomposition, coalition formation, allocation,
and subsequent execution through robot skills. Its conclusion lists dynamic task
allocation as future work. We use its structured task-planning motivation; our
single interpretation call and numerical allocation do not reproduce its full
multi-stage prompting or heterogeneous coalition formation.

Source: https://arxiv.org/html/2309.10062v2 (methodology IV and conclusion VII).

### COHERENT

Liu et al., *COHERENT: Collaboration of Heterogeneous Multi-Robot System with Large
Language Models*, arXiv:2409.15146v2.

Its PEFA mechanism cycles between a centralized LLM task assigner and LLM-backed
robot executors, using feedback and history to adjust plans. The paper retains
five recent dialogue iterations for token efficiency. This is richer than
simple task reassignment, and is not accurately characterized as an LLM reading
every simulator timestep. We borrow the execution-feedback-adjustment concept,
while implementing limited feedback and replacement in local code.

Source: https://arxiv.org/html/2409.15146v2 (methodology III).

These papers are foundations, not reproduced baselines or evidence that this
project outperforms them. Their manipulation/heterogeneous-task scope is broader
than the current navigation-visit demonstrator. Novelty and cost/success
comparisons require controlled experiments and a broader literature review.

## Our implementation

```text
Natural-language instruction
    -> one LLM interpretation invocation
    -> validated station visits / explicit robot requests
    -> deterministic route-distance allocation
    -> fleet execution and local progress monitoring
           success -> completed
           stalled with known pose -> bounded local reassignment -> execution
           unreliable pose/sensing -> fleet-wide stop
```

Neither allocation, navigation, routine status monitoring, nor recovery calls the
LLM. A new/clarified user instruction is a new interpretation request. Invalid
or ambiguous requests stop for clarification rather than silently requesting
another generated plan. Provider/SDK network retries can still happen; one
application invocation is not an assurance of exactly one HTTP request or charge.

## What “closest robot” means

For an unassigned job, the current allocator minimizes:

`candidate cost = existing queued route metres + route metres for the new job`

Route length comes from A*, including wall detours and configured clearance.
When both robots have no queue this favors the shortest traversable route, which
may differ from straight-line proximity. A nearby busy robot can lose to a more
distant idle robot. Explicit user assignments are preserved initially; successful
recovery can override them as authorized by the replacement policy.

Jobs are assigned greedily, with explicit requests processed first. This is not a
global minimum-distance or minimum-makespan solver. Equal robot speed is assumed
when using metres as a workload proxy. Distance does not predict traffic waits,
battery limits, robot capabilities, or all execution failures. Replacement uses
current positions, peer/live obstacles, and existing queue commitments.

## API-cost evidence

`allocate_tasks.py` writes `llm_usage.json` alongside its mission:

- Offline intent mode: zero interpretation invocations.
- Natural-language mode: one application-level interpretation invocation on success.
- Execution/recovery: zero LLM invocations by architecture; recovery integration
  testing rejects construction of a Gemini client.
- Model, interpretation latency, and provider-reported prompt/output/total token
  counts when available. Missing counts remain null, not zero.

No monetary savings have been measured yet. To evaluate them, compare the same
instructions, initial scenes, failure injections, and model settings across:

1. Single interpretation with recovery disabled (`--max-reassignments 0`).
2. Single interpretation with deterministic recovery (current design).
3. A separately implemented LLM-replanning recovery baseline.

Measure task completion, recovery completion, route travel, makespan, wait time,
LLM invocations, tokens, and billed cost under the model's actual pricing.
Keep retries and rejected instructions in the accounting. A local-feedback
ablation is not equivalent to reproducing all of SMART-LLM or COHERENT.

## Current evidence and next milestone

The user verified two-robot fleet completion and the simulated-stall replacement
trial. Saved recovery logs show Robot B completed both its own task and Robot A's
transferred task, with one reassignment. Offline tests cover distance-aware
allocation and recovery without LLM calls. Live natural-language interpretation
with the configured Gemini account remains the next test, followed by repeated
allocation/recovery experiments and scaling to 5–10 robots.
