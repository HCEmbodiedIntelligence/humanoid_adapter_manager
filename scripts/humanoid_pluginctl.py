#!/usr/bin/env python3
"""CLI used by a local Web backend to manage no-build robot plugins."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from humanoid_manager.deployment import (
    DEFAULT_PLUGIN_ROOT,
    DeploymentError,
    deploy_archive,
    json_text,
    list_deployed,
    pack_directory,
    resolve_robot_deployment,
    validate_archive,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", "--plugin-root", type=Path, default=DEFAULT_PLUGIN_ROOT)
    parser.add_argument("--state-root", type=Path)
    subparsers = parser.add_subparsers(dest="command", required=True)

    validate = subparsers.add_parser("validate")
    validate.add_argument("archive", type=Path)
    validate.add_argument("--skip-link-check", action="store_true")

    pack = subparsers.add_parser("pack")
    pack.add_argument("source", type=Path)
    pack.add_argument("output", type=Path)

    deploy = subparsers.add_parser("deploy")
    deploy.add_argument("archive", type=Path)
    deploy.add_argument("--skip-link-check", action="store_true")

    subparsers.add_parser("list")
    subparsers.add_parser("web", help="Read one structured configuration request from stdin")

    resolve = subparsers.add_parser("resolve")
    resolve.add_argument("robot_id")
    return parser


def main() -> int:
    arguments = _parser().parse_args()
    try:
        if arguments.command == "web":
            from humanoid_manager.configuration import ConfigurationManager
            manager = ConfigurationManager(arguments.root, arguments.state_root or arguments.root / ".configurator")
            payload = sys.stdin.read(16 * 1024 * 1024 + 1)
            if len(payload.encode()) > 16 * 1024 * 1024:
                raise DeploymentError("配置请求超过大小限制")
            result = manager.dispatch(json.loads(payload))
        elif arguments.command == "validate":
            manifest = validate_archive(
                arguments.archive,
                check_linkage=not arguments.skip_link_check,
            )
            result = {
                "status": "valid",
                "artifact_type": manifest["artifact_type"],
            }
            if manifest["artifact_type"] == "plugin":
                result.update(
                    plugin_type=manifest["plugin_type"],
                    plugin_id=manifest["plugin_id"],
                )
            else:
                result["robot_id"] = manifest["robot_id"]
        elif arguments.command == "pack":
            result = {
                "archive": str(
                    pack_directory(arguments.source, arguments.output)
                )
            }
        elif arguments.command == "deploy":
            from humanoid_manager.runtime_state import deployment_lock
            with deployment_lock(arguments.root):
                result = {"deployed": str(deploy_archive(arguments.archive, arguments.root,
                    check_linkage=not arguments.skip_link_check)), "restart_required": True}
        elif arguments.command == "list":
            result = list_deployed(arguments.root)
        else:
            deployment = resolve_robot_deployment(
                arguments.root,
                arguments.robot_id,
            )
            result = deployment.as_dict()
        print(json_text(result))
        return 0
    except (DeploymentError, OSError, ValueError, TypeError, KeyError) as error:
        print(json_text({"status": "error", "message": str(error), "kind": type(error).__name__}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
