# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.

from clearpath_config.common.types.platform import Platform
from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.platform.platform import DescriptionPackagePath
import os


class PlatformDescription():

    class BasePlatform():
        def __init__(
                self,
                package: str,
                path: str,
                file: str,
                macro: str = None,
                parameters: dict = None) -> None:
            self.package = package
            self.path = path
            self.file = file
            self.macro = macro
            self.parameters = parameters

    class ClearpathPlatform(BasePlatform):
        pkg_clearpath_platform_description = 'clearpath_platform_description'

        def __init__(self, config: ClearpathConfig) -> None:
            super().__init__(
                package=self.pkg_clearpath_platform_description,
                path=f'urdf/{config.get_platform_model()}/',
                file=config.get_platform_model(),
                macro=config.get_platform_model(),
                parameters={
                    'wheel': config.platform.wheel,
                }
            )

    class GenericPlatform(BasePlatform):
        def __init__(self, config: ClearpathConfig) -> None:
            description = config.platform.description
            package = description[DescriptionPackagePath.PACKAGE]
            path = description[DescriptionPackagePath.PATH]
            macro = description[DescriptionPackagePath.MACRO]
            super().__init__(
                package=package,
                file=os.path.basename(path),
                path=os.path.dirname(path)+"/",
                macro=macro,
                parameters=None
            )

    def __new__(cls, model: Platform, config: ClearpathConfig) -> BasePlatform:
        if model == Platform.GENERIC:
            return PlatformDescription.GenericPlatform(config)
        else:
            return PlatformDescription.ClearpathPlatform(config)
