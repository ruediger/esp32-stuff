#!/usr/bin/python
"""Generate C++ headers pre-rendered monospace glyphs.

Glyphs are internally represented as text strings and turned into
bitmaps that match the SSD1306 screen buffer.
"""

import argparse
from typing import List, Sequence

_GLYPHS = ["""\
 XXX
X   X
X X X
X   X
 XXX """, """\
  X
 XX
  X
  X
 XXX """, """\
 XXX
X   X
   X
  X
XXXX """, """\
 XXX
    X
 XXX
    X
 XXX """, """\
  XX
 X X
XXXXX
   X
   X """, """\
 XXX
X
 XXX
    X
 XXX """, """\
 XXX
X
 XXX
X   X
 XXX """, """\
 XXXX
   X
  X
 X
 X   """, """\
 XXX
X   X
 XXX
X   X
 XXX """, """\
 XXX
X   X
 XXX
    X
 XXX """, """\
 XXX
X   X
XXXXX
X   X
X   X""", """\
XXXX
X   X
XXXX
X   X
XXXX """, """\
 XXX
X   X
X
X   X
 XXX """, """\
XXXX
X   X
X   X
X   X
XXXX """, """\
XXXXX
X   
XXXXX
X   
XXXXX""", """\
XXXXX
X
XXXXX
X
X    """, """\
 XXX
X   
X  XX
X   X
 XXX """, """\
X   X
X   X
XXXXX
X   X
X   X""", """\
 XXX
  X
  X
  X
 XXX """, """\
 XXXX
    X
    X
 X  X
  XX """, """\
X  X
X X
XX
X X
X  X """, """\
 X
 X
 X
 X
 XXX """, """\
 X X
X X X
X X X
X X X
X X X""", """\
 X  X
XX  X
X X X
X  XX
X  XX""", """\
 XXX
X   X
X   X
X   X
 XXX """, """\
XXXX
X   X
XXXX
X
X    """, """\
 XXX
X   X
X X X
X  XX
 XXXX""", """\
XXXX
X   X
XXXX
X   X
X   X""", """\
 XXX
X
 XXX
    X
 XXX """, """\
XXXXX
  X
  X
  X
  X  """, """\
X   X
X   X
X   X
X   X
 XXX """, """\
X   X
X   X
X   X
 X X
  X  """, """\
X   X
X   X
X X X
X X X
 X X """, """\
X   X
 X X
  X
 X X
X   X""", """\
X   X
X   X
 X X
  X
  X  """, """\
XXXXX
    X
  X
 X
XXXXX""", """\




     """, """\
  X
  X
  X

  X  """, """\
  X
   X
  X

  X  """, """\

  X

  X
     """, """\

  X

  X
 X   """, """\




  X  """, """\



  X
 X   """, """\


 XXX

     """, """\

  X
 XXX
  X
     """, """\
    X
 X X
  X
 X X
X    """]

_TEMPLATE = """\
#ifndef FONT{size}_HPP
#define FONT{size}_HPP

namespace font{size} {{
  const size_t width = {width};
  const size_t height = {height};
  const uint8_t glyphs[][{length}] = {data};
}}

#endif\
"""

## TODO -> font{size} width, height, glyphs

def split_glyph(d: str) -> (int, int, List[str]):
    """Split glyph and return width, height, and glyph broken into lines."""
    lines = d.split('\n')
    if len(lines) == 0:
        return (0, 0, [])
    while len(lines[0]) == 0:  # Remove empty frist line if char starts with \n
        lines.pop(0)
    height = len(lines)
    width = max([len(line) for line in lines])
    return (width, height, lines)

def set_pixel(buffer: Sequence[int], width: int, x: int, y: int) -> None:
    """Set pixel at x,y in buffer with width."""
    buffer[x + y//8 * width] |= 1 << (y & 7)

def buffer_size(width: int, height: int) -> int:
    """Return buffer size."""
    return width * ((height + 7) // 8)

def to_buffer(d: str) -> List[int]:
    """Convert text representation into binary (SSD1306) format."""
    width, height, lines = split_glyph(d)
    buffer = [0] * buffer_size(width, height)
    x = 0
    y = 0
    for line in lines:
        for c in line:
            if c != ' ':
                set_pixel(buffer, width, x, y)
            x += 1
        y += 1
        x = 0
    return buffer

def format_seq(buffer: Sequence[int]) -> str:
    """Format sequence as C++ binary array."""
    return '{' + ', '.join([f'0b{i:08b}' for i in buffer]) + '}'

def render_template(glyphs: Sequence[str]) -> str:
    """Formats template using glyphs."""
    width, height, _ = split_glyph(glyphs[0])

    data = [format_seq(to_buffer(d)) for d in glyphs]

    return _TEMPLATE.format(
        data='{\n    ' + ',\n    '.join(data) + '\n  }',
        size=f'{width}x{height}',
        width=width,
        height=height,
        length=buffer_size(width, height))

def resize(d: str, multiply: int = 2) -> str:
    """Resize a string."""
    return '\n'.join([
        new_line
        for line in d.split('\n')
        for doubled_line in [''.join([c*multiply for c in line])]
        for new_line in (doubled_line, )*multiply
    ])

def main() -> None:
    """Main function."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--multiplier", help="Size multiplier for font (int).")
    args = parser.parse_args()
    multiplier = int(args.multiplier) if args.multiplier else 1
    print(render_template([resize(g, multiplier) for g in _GLYPHS]))

if __name__ == "__main__":
    main()
