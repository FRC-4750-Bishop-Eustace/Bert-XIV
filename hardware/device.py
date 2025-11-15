# MIT License
#
# Copyright (c) 2025 Beʳᵗ FRC Team 4750
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from abc import ABC
from typing import Dict, Type, Any

class Device(ABC):
    registry: Dict[str, Dict[str, Type["Device"]]] = {}

    @classmethod
    def RegisterBackend(cls, deviceType: str, backendName: str, backendClass: Type["Device"]) -> None:
        deviceType = deviceType.lower()
        backendName = backendName.lower()
        cls.registry.setdefault(deviceType, {})[backendName] = backendClass

    @classmethod
    def GetBackendClass(cls, deviceType: str, backendName: str) -> Type["Device"]:
        deviceType = deviceType.lower()
        backendName = backendName.lower()
        try:
            return cls.registry[deviceType][backendName]
        except KeyError as e:
            raise ValueError(f"No backend registered for \'{deviceType}\' -> \'{backendName}\'") from e

    @classmethod
    def Create(cls, deviceType: str, backendName: str, *args, **kwargs) -> "Device":
        backendClass = cls.GetBackendClass(deviceType, backendName)
        return backendClass(*args, **kwargs)

    @classmethod
    def CreateFromConfig(cls, config: dict) -> "Device":
        deviceType = config.get("device_type") or config.get("type_hint") or config.get("kind")
        if deviceType is None:
            raise ValueError("No device type specified in JSON config (\"device_type\" or \"type_hint\" or \"kind\")")

        vendor = config.get("vendor") or config.get("backend") or config.get("manufacturer")
        if vendor is None:
            raise ValueError("No vendor specified in JSON config (\"vendor\" or \"backend\" or \"manufacturer\")")

        typ = config.get("type") or config.get("model")
        if typ is None:
            raise ValueError("No type specified in JSON config (\"type\" or \"model\")")

        params = config.get("params", {}) or config.get("parameters", {})

        backendName = f"{vendor}_{typ}".replace(" ", "").lower()

        return cls.Create(deviceType, vendor, typ, **params)
