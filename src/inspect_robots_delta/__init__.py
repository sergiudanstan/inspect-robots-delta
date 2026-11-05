"""Inspect Robots plugin: a simulated 3-RRR delta robot with suction pick-and-place."""

from __future__ import annotations

from inspect_robots.scene import Scene
from inspect_robots.scorer import success_at_end
from inspect_robots.task import Task

from inspect_robots_delta.policies import DeltaJointScriptedPolicy, DeltaScriptedPolicy
from inspect_robots_delta.sim import DeltaSimEmbodiment, DeltaSimJointEmbodiment

__all__ = [
        "DeltaJointScriptedPolicy",
        "DeltaScriptedPolicy",
        "DeltaSimEmbodiment",
        "DeltaSimJointEmbodiment",
        "delta_pick_place",
]


def delta_pick_place(num_scenes: int = 10, max_steps: int = 300) -> Task:
        """Pick the puck and drop it in the bin, over seeded layouts."""
        return Task(
                name="delta-pick-place",
                scenes=[
                        Scene(
                                id=f"layout-{i}",
                                instruction="pick up the green puck and drop it in the blue bin",
                                init_seed=i,
                        )
                        for i in range(num_scenes)
                ],
                scorer=success_at_end(),
                max_steps=max_steps,
        )
