# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
import glob
import os

from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.files import copy, get, rmdir
from conan.tools.scm import Version

SUPPORTED_OS = ("Linux", "Macos", "Windows")
SUPPORTED_ARCH = ("x86_64", "armv8")


class OverlumeConan(ConanFile):
    name = "overlume"
    license = "Apache-2.0"
    url = "https://github.com/amerghazal7/overlume"
    description = "Real-time rendering library (Filament) behind a POD-only C++ API. Prebuilt release binaries."
    package_type = "library"
    settings = "os", "arch", "compiler"
    options = {"shared": [True, False]}
    default_options = {"shared": True}

    def validate(self):
        if str(self.settings.os) not in SUPPORTED_OS or str(self.settings.arch) not in SUPPORTED_ARCH:
            raise ConanInvalidConfiguration(f"no overlume package for {self.settings.os}/{self.settings.arch}")
        if self.settings.os == "Windows" and (
                str(self.settings.compiler) not in ("msvc", "clang")
                or self.settings.compiler.get_safe("runtime") == "static"):
            raise ConanInvalidConfiguration(
                "overlume on Windows ships MSVC /MD binaries; use compiler=msvc with compiler.runtime=dynamic")
        if self.settings.os == "Linux" and not self.options.shared:
            # The static archive embeds objects built against libc++ 18.
            ok = (self.settings.compiler == "clang" and self.settings.compiler.get_safe("libcxx") == "libc++"
                  and Version(self.settings.compiler.version) >= 18)
            if not ok:
                raise ConanInvalidConfiguration(
                    "overlume static on Linux needs clang >= 18 with compiler.libcxx=libc++; "
                    "use -o overlume/*:shared=True with gcc")

    def package_id(self):
        if self.info.options.shared:
            del self.info.settings.compiler  # the shared library carries its own C++ runtime

    def build(self):
        src = self.conan_data["sources"][self.version][str(self.settings.os)][str(self.settings.arch)]
        get(self, src["url"], sha256=src["sha256"], destination=os.path.join(self.build_folder, "dl"))

    def package(self):
        root = os.path.join(self.build_folder, "dl")
        entries = os.listdir(root)
        if len(entries) == 1:  # the Linux tar.gz has a top-level directory; macOS/Windows are flat
            root = os.path.join(root, entries[0])
        for sub in ("include", "share"):
            copy(self, "*", os.path.join(root, sub), os.path.join(self.package_folder, sub))
        lib, dst = os.path.join(root, "lib"), os.path.join(self.package_folder, "lib")
        if self.options.shared:
            for pattern in ("liboverlume.so*", "liboverlume*.dylib", "overlume.lib"):
                copy(self, pattern, lib, dst)
            copy(self, "overlume.dll", os.path.join(root, "bin"), os.path.join(self.package_folder, "bin"))
        else:
            for pattern in ("liboverlume.a", "overlume_static.lib"):
                copy(self, pattern, lib, dst)
            copy(self, "*", os.path.join(lib, "overlume"), os.path.join(dst, "overlume"))

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "overlume")
        self.cpp_info.set_property(
            "cmake_target_name", "overlume::overlume" if self.options.shared else "overlume::overlume_static")
        if self.options.shared:
            self.cpp_info.libs = ["overlume"]
            return
        os_name = str(self.settings.os)
        deps = sorted(glob.glob(os.path.join(self.package_folder, "lib", "overlume", "deps", "*")))
        self.cpp_info.libs = ["overlume_static" if os_name == "Windows" else "overlume"]
        # Link items go through system_libs: Conan emits those after the library itself, so the
        # archives it needs come later on the command line (exelinkflags would put them first).
        if os_name == "Linux":
            link = ["-Wl,--allow-multiple-definition"]
            self.cpp_info.system_libs = ["-Wl,--start-group", *deps, "-Wl,--end-group",
                                         "EGL", "GLESv2", "dl", "pthread", "m", "rt"]
        elif os_name == "Macos":
            link = []
            self.cpp_info.frameworks = ["Metal", "QuartzCore", "CoreVideo", "IOSurface", "Foundation", "Cocoa",
                                        "OpenGL", "SystemConfiguration", "CoreFoundation", "CoreServices"]
            self.cpp_info.system_libs = [*deps, "dl", "pthread"]
        else:
            link = ["/FORCE:MULTIPLE", "-ignore:4221"]
            self.cpp_info.system_libs = [*deps, "opengl32", "gdi32", "user32", "shell32", "advapi32", "ws2_32",
                                         "bcrypt", "crypt32", "secur32", "iphlpapi", "shlwapi", "ole32",
                                         "windowscodecs"]
        self.cpp_info.exelinkflags = link
        self.cpp_info.sharedlinkflags = link
