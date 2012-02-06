Description:
	
	Shared library holding common code used in various projects 
	from www.joesfer.com.	
	
	RenderLib provides classes specifically related to graphics: geometry, 
	data structures, algebra, etc.
	
License:

	This software is released under the LGPL-3.0 license: 
	http://www.opensource.org/licenses/lgpl-3.0.html	

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
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  
	02110-1301  USA
	
Compilation:

	- RenderLib depends on CoreLib, so we'll build it first:

		mkdir <parent_folder>/<corelib_folder>
		cd <parent_folder>/<corelib_folder>
		git clone git://github.com/joesfer/CoreLib.git 

		mkdir .build
		cd .build
		cmake .. 
		
		On Windows: cmake will generate a Visual studio solution on .build
		On Linux: cmake will generate a GCC makefile
		
		build the generated makefile/solution. If the compilation 
		succeeded, a static library should have been generated under
		<parent_folder>/<corelib_folder>/lib/<build_type>

	- Now we'll build RenderLib:

		mkdir <parent_folder>/<renderlib_folder>
		cd <parent_folder>/<renderlib_folder>
		git clone git://github.com/joesfer/RenderLib.git 

		mkdir .build
		cd .build
		cmake .. 

		Note that by default, CoreLib is expected to be in <parent_folder>
		(that is, at the same level as RenderLib). However, this can be
		changed by setting the CORELIB_INCLUDE_DIR and CORELIB_LIB_DIR
		variables when calling CMake.
		
		On Windows: cmake will generate a Visual studio solution on .build
		On Linux: cmake will generate a GCC makefile
		
		build the generated makefile/solution. If the compilation 
		succeeded, a static library should have been generated under
		<parent_folder>/<renderlib_folder>/lib/<build_type>