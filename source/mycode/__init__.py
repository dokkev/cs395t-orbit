# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configurations for the object lift environments."""

from .reach import *  # noqa
# We leave this file empty since we don't want to expose any configs in this package directly.
# We still need this file to import the "config" module in the parent package.

# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Package containing task implementations for various robotic environments."""

import os
import toml

print("=============================init_orbit_tasks==========================")

# Conveniences to other module directories via relative paths
ORBIT_TASKS_EXT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../extensions/omni.isaac.orbit_tasks/"))
"""Path to the extension source directory."""

ORBIT_TASKS_METADATA = toml.load(os.path.join(ORBIT_TASKS_EXT_DIR, "config", "extension.toml"))
"""Extension metadata dictionary parsed from the extension.toml file."""

# Configure the module-level variables
__version__ = ORBIT_TASKS_METADATA["package"]["version"]

##
# Register Gym environments.
##

from .utils import import_packages

# The blacklist is used to prevent importing configs from sub-packages
_BLACKLIST_PKGS = ["utils"]
# Import all configs in this package
import_packages(__name__, _BLACKLIST_PKGS)
