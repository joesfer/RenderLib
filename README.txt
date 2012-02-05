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

	- Clone the git repository into a local folder:

		mkdir <renderlib_folder>
		cd <renderlib_folder>
		git clone git://github.com/joesfer/RenderLib.git 

	- The project depends on the CoreLib library, included as a submodule.
	
		in <renderlib_folder>
		git submodule init
		git submodule update
		
		git should print "Cloning into 'src/CoreLib'..."
		Otherwise, add it manually by doing:
		
		git submodule add git://github.com/joesfer/CoreLib.git source/CoreLib
		git submodule update
		
	- Build RenderLib
	
		cd <renderlib_folder>
		mkdir .build
		cd .build
		cmake .. 
		
		Under windows: cmake will generate a Visual studio solution on .build
		Under linux: cmake will generate a GCC makefile

		Note that there is a dependency with CoreLib, which will be built first.
		If everything went well, after building RenderLib a new folder structure 
		<renderlib_folder>/lib/<build_type> containing the static library 
		should have been generated.
