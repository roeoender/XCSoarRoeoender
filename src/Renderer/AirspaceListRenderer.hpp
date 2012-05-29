/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
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

#ifndef XCSOAR_AIRSPACE_LIST_RENDERER_HPP
#define XCSOAR_AIRSPACE_LIST_RENDERER_HPP

#include "Screen/Point.hpp"

class Canvas;
class AbstractAirspace;
struct DialogLook;
struct AirspaceLook;
struct AirspaceRendererSettings;

namespace AirspaceListRenderer
{
  gcc_pure
  UPixelScalar GetHeight(const DialogLook &dialog_look);

  void Draw(Canvas &canvas, const PixelRect rc, const AbstractAirspace &airspace,
            const DialogLook &dialog_look, const AirspaceLook &look,
            const AirspaceRendererSettings &renderer_settings);
}

#endif
