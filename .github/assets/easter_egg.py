#!/usr/bin/env python3
# Copyright (c) 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from asciimatics.effects import Print, Stars, Effect
from asciimatics.renderers import ColourImageFile, FigletText, SpeechBubble
from asciimatics.scene import Scene
from asciimatics.screen import Screen
from asciimatics.exceptions import ResizeScreenError
import os
import sys
import re

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)


class Background(Effect):
    def __init__(self, screen, **kwargs):
        super().__init__(screen, **kwargs)

    def reset(self):
        self._screen.clear_buffer(7, 0, 0)

    def _update(self, frame_no):
        self._screen.clear_buffer(7, 0, 0)

    @property
    def stop_frame(self):
        return self._stop_frame


def display(screen):
    """
    Display the easter egg.
    """
    scenes = []
    renderer = ColourImageFile(
        screen, "wave.gif", screen.height, uni=screen.unicode_aware
    )
    renderer._index = len(renderer._images) // 2

    new_images = []
    for image in renderer._images:
        new_image = re.sub(r"\$\{\d+,2,\d+\}\.", " ", image)
        new_images.append(new_image)
    renderer._images = new_images

    effects = [
        Background(screen),
        Print(
            screen,
            FigletText("COUGARS", font="banner3" if screen.width > 80 else "banner"),
            screen.height // 4,
            colour=7,
            bg=7 if screen.unicode_aware else 0,
            speed=1,
            transparent=True,
        ),
        Print(screen, renderer, 0, speed=1, transparent=True),
    ]
    scenes.append(Scene(effects, -1))
    screen.play(scenes, stop_on_resize=True, repeat=False)


if __name__ == "__main__":
    while True:
        try:
            Screen.wrapper(display)
            sys.exit(0)
        except ResizeScreenError:
            pass
