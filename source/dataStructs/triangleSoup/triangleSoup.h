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

#include <math/algebra/point/point3.h>

namespace RenderLib {
namespace DataStructures {

/* 
	=============================================================================
	ITriangleSoup interface - abstract interface used to pass geometry 
	information around. The actual implementation may vary depending on the
	data source (e.g. a maya plugin, a standalone application, an obj parser...)
	=============================================================================
*/
	template< typename VertexType >
	class ITriangleSoup {
	public:
		virtual size_t		numIndices() const = 0;
		virtual const int*	getIndices() const = 0;

		virtual size_t		numVertices() const = 0;
		virtual const VertexType*	getVertices() const = 0;
		
		virtual RenderLib::Geometry::BoundingBox	calculateBounds() const = 0;
	};
}
}