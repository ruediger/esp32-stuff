#ifndef FONT_HPP
#define FONT_HPP

struct font {
  size_t width;
  size_t height;
  size_t no_of_glyphs;
  const uint8_t **glyphs;
};

#endif
