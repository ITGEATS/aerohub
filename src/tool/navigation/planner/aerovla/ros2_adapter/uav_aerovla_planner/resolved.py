"""Resolved task-config loading (ROS-free).

The planner framework hands each spawned planner a resolved YAML
(``{resolved_dir}/{task_id}.yaml``) built by ``PlannerRegistry.resolve``:
task_id, planner parameters, vehicle list and the per-vehicle deployment
(see ``src/tool/navigation/framework/uav_planner_framework/registry.py``
L100-128 and ``ros_node.py`` L147-207). The AeroVLA planner reads exactly
this file so that segments it publishes carry the framework's task_id and
vehicle set -- ``OutputGateway.accept`` rejects mismatched ids
(``trajectory.py`` L108-140).
"""

import yaml


class ResolvedConfigError(ValueError):
    pass


def load_resolved(path):
    """Load and validate a resolved config; returns the minimal view the
    planner needs: task_id, vehicle_id, deployment namespace and the
    planner parameters mapping."""
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ResolvedConfigError("resolved config must be a mapping: " + str(path))
    task = data.get("task") or {}
    task_id = task.get("task_id")
    if not task_id or not isinstance(task_id, str):
        raise ResolvedConfigError("resolved.task.task_id missing: " + str(path))
    planner = data.get("planner") or {}
    parameters = planner.get("parameters")
    if not isinstance(parameters, dict):
        raise ResolvedConfigError("resolved.planner.parameters missing: " + str(path))
    vehicles = data.get("vehicles")
    if not isinstance(vehicles, list) or not vehicles or \
            any(not isinstance(v, str) or not v for v in vehicles):
        raise ResolvedConfigError("resolved.vehicles must be a nonempty string list: "
                                  + str(path))
    deployment = data.get("deployment") or {}
    namespace = ""
    if isinstance(deployment.get(vehicles[0]), dict):
        namespace = str(deployment[vehicles[0]].get("namespace", vehicles[0]))
    return {
        "task_id": task_id,
        "vehicle_id": vehicles[0],
        "namespace": namespace,
        "parameters": parameters,
    }
