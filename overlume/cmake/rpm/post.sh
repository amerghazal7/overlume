#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
# RPM distros search /usr/lib64 (not /usr/lib) for 64-bit libraries; the package
# installs one libdir (/usr/lib), so register it with the loader.
echo /usr/lib > /etc/ld.so.conf.d/overlume.conf
/sbin/ldconfig
