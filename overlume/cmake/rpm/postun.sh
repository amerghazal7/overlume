#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
# Last package version removed: drop the loader entry (upgrade keeps it).
if [ "$1" = 0 ]; then rm -f /etc/ld.so.conf.d/overlume.conf; fi
/sbin/ldconfig
