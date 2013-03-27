/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2013 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Screen/Bitmap.hpp"
#include "Screen/Custom/LibJPEG.hpp"
#include "Screen/Custom/UncompressedImage.hpp"
#include "UncompressedImage.hpp"

#include <jpeglib.h>

#include <setjmp.h>

struct JPEGErrorManager {
  struct jpeg_error_mgr base;

  jmp_buf setjmp_buffer;

  JPEGErrorManager() {
    base.error_exit = ErrorExit;
  }

  gcc_noreturn
  void ErrorExit() {
    longjmp(setjmp_buffer, 1);
  }

  gcc_noreturn
  static void ErrorExit(j_common_ptr cinfo) {
    JPEGErrorManager *err = reinterpret_cast<JPEGErrorManager *>(cinfo);
    err->ErrorExit();
  }
};

bool
Bitmap::LoadFile(const TCHAR *path)
{
  const UncompressedImage uncompressed = LoadJPEGFile(path);
  texture = ImportTexture(uncompressed);
  if (texture == nullptr)
    return false;

  size = { uncompressed.GetWidth(), uncompressed.GetHeight() };
  return true;
}
