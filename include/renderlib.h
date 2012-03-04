/*
	================================================================================
	This software is released under the LGPL-3.0 license: http://www.opensource.org/licenses/lgpl-3.0.html

	Copyright (c) 2012, Jose Esteve. http://www.joesfer.com

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 3.0 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
	================================================================================
*/

#pragma once

/// Main header

#include <math/constants.h>

#include <math/algebra/matrix/matrix3.h>
#include <math/algebra/matrix/matrix4.h>
#include <math/algebra/matrix/matrix5.h>
#include <math/algebra/point/point2.h>
#include <math/algebra/point/point3.h>
#include <math/algebra/vector/vector2.h>
#include <math/algebra/vector/vector3.h>
#include <math/algebra/normal/normal2.h>
#include <math/algebra/normal/normal3.h>
#include <math/algebra/quaternion/quaternion.h>

#include <math/sampling/random.h>
#include <math/sampling/lowDiscrepancy.h>

#include <geometry/bounds/boundingBox.h>
#include <geometry/bounds/bounds2D.h>
#include <geometry/intersection/intersection.h>
#include <geometry/utils.h>
#include <geometry/tessellation/delaunay/delaunay2D.h>
#include <geometry/tessellation/delaunay/delaunay3D.h>

#include <dataStructs/photonMap/photonMap.h>
#include <dataStructs/triangleSoup/triangleSoup.h>
#include <dataStructs/kdtree/kdTree.h>
#include <dataStructs/bvh/bvh.h>

#include <raytracing/ray/ray.h>