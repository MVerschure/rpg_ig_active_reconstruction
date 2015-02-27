/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
* 
combined_relative_movement is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
combined_relative_movement is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with combined_relative_movement. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "utils/relative_movement.h"
#include "utils/kinematic_movement_description.h"

namespace st_is
{
class CombinedKinematicMovementDescription;

/// Class to hold a series of fixed relative movements
class CombinedRelativeMovement
{
public:
  friend CombinedKinematicMovementDescription;
  
  CombinedRelativeMovement();
  
  /** applies the relative movement queue to a base pose */
  st_is::GeometryPose applyToBasePose( st_is::GeometryPose& _base );
  
  /** replaces the current relative movement chain represented by the object with _to_equal as the one, single chain element */
  CombinedRelativeMovement& operator=( RelativeMovement const& _to_equal );
  
  /** creates a new combined relative movement with the same relative movement chain as the current object but with _to_add appended */
  CombinedRelativeMovement operator+( CombinedRelativeMovement const& _to_add );
  /** creates a new combined relative movement with the same relative movement chain as the current object but with _to_add appended */
  CombinedRelativeMovement operator+( RelativeMovement const& _to_add );
  /** creates a new combined relative movement with the same relative movement chain as the current object but with _to_add appended */
  CombinedKinematicMovementDescription operator+( KinematicMovementDescription const& _to_add );
  /** creates a new combined relative movement with the same relative movement chain as the current object but with _to_add appended */
  CombinedKinematicMovementDescription operator+( CombinedKinematicMovementDescription const& _to_add );
  
  /** appends _to_add to the internal relative movement chain */
  CombinedRelativeMovement& operator+=( CombinedRelativeMovement const& _to_add );
  /** appends _to_add to the internal relative movement chain */
  CombinedRelativeMovement& operator+=( RelativeMovement const& _to_add );
private:
  std::deque< RelativeMovement > relative_movement_queue_;
}
  
}