#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from rclpy.node import Node


def _normalize_fqn(name: str) -> str:
    cleaned = name.strip()
    if not cleaned:
        return cleaned
    if not cleaned.startswith("/"):
        cleaned = "/" + cleaned
    return cleaned.rstrip("/") or "/"


def _node_fqns(node: Node) -> set[str]:
    results = set()
    for name, namespace in node.get_node_names_and_namespaces():
        ns = namespace.rstrip("/")
        if not ns:
            ns = "/"
        if ns == "/":
            results.add(_normalize_fqn(name))
        else:
            results.add(_normalize_fqn(f"{ns}/{name}"))
    return results


def _service_fqns(node: Node) -> set[str]:
    return {
        _normalize_fqn(name)
        for name, _types in node.get_service_names_and_types()
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Wait until required ROS graph nodes/services are visible.")
    parser.add_argument("--label", default="stage", help="Human-readable stage label")
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--poll-sec", type=float, default=0.2)
    parser.add_argument("--node", action="append", default=[], dest="nodes")
    parser.add_argument("--service", action="append", default=[], dest="services")
    args = parser.parse_args()

    expected_nodes = [_normalize_fqn(name) for name in args.nodes if name.strip()]
    expected_services = [_normalize_fqn(name) for name in args.services if name.strip()]

    if not expected_nodes and not expected_services:
        print(f"[wait_for_graph_entities] {args.label}: nothing to wait for", flush=True)
        return 0

    rclpy.init(args=None)
    node = Node("layer_1_bringup_waiter")
    deadline = time.monotonic() + max(args.timeout_sec, 0.0)

    try:
        while time.monotonic() <= deadline:
            seen_nodes = _node_fqns(node)
            seen_services = _service_fqns(node)

            missing_nodes = [name for name in expected_nodes if name not in seen_nodes]
            missing_services = [name for name in expected_services if name not in seen_services]

            if not missing_nodes and not missing_services:
                print(
                    f"[wait_for_graph_entities] {args.label}: ready "
                    f"(nodes={len(expected_nodes)}, services={len(expected_services)})",
                    flush=True,
                )
                return 0

            time.sleep(max(args.poll_sec, 0.05))

        details = []
        if missing_nodes:
            details.append("missing nodes: " + ", ".join(missing_nodes))
        if missing_services:
            details.append("missing services: " + ", ".join(missing_services))
        print(
            f"[wait_for_graph_entities] {args.label}: timed out after "
            f"{args.timeout_sec:.1f}s; " + "; ".join(details),
            file=sys.stderr,
            flush=True,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
