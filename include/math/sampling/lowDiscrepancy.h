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

namespace RenderLib {
namespace Math {

	/*
		Sampling with Hammersley and Halton Points
		source http://www.cse.cuhk.edu.hk/~ttwong/papers/udpoint/udpoint.pdf

		The Hammersley and Halton point sets, two well known low discrepancy
		sequences, have been used for quasi-Monte Carlo integration in previous research.
		A deterministic formula generates a uniformly distributed and stochastic-looking
		sampling pattern, at low computational cost. The Halton point set is also useful for
		incremental sampling.

		The Hammersley point set with p1 = 2 gives the most uniformly distributed sampling
		pattern. For higher p1, the points tend to align and reduce its usefulness. Although
		the Halton point sets do not give patterns as uniformly distributed as Hammersley
		point sets, they do not have the line-up problem and it allows incremental sampling.
	*/

	// Hammersley Points on 2D Plane with p1 = 2
	void planeHammersley(float *result, int n);

	// Halton Points on 2D Plane with p1 = 2
	void planeHalton(float *result, int n, int p2);

	// Hammersley Points on Sphere with p1 = 2
	void sphereHammersley(float *result, int n);

	// Halton Points on Sphere with p1 = 2
	void sphereHalton(float *result, int n, int p2 = 7 );

} // namespace Math
} // namespace RenderLib