#!/usr/bin/env bash
# Disable the stale anyio pytest plugin in this environment.
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 exec python3 -m pytest "$@"
