Astrodynamics Program:
Visually represents a replica solar system with nothing but C++, OpenGL and GLFW libraries.
Rendering, shadow mapping, vertex handling all done by hand.
Spheres approximated with triangles and texture laying by hand, no models, no copy + paste formulae.
Matrix manipulation of positions of objects, and camera all done by hand, object positions
estimated using invented data with 2-body Einsteinian relativity - with the future capability
to handle PPN approximations with NASA figures (given below).

Celestial body parameters were determined as follows:
Solar radius is 10 units across, every other body in the solar system
is the appropriate scaled radius comparatively.
Solar mass is 10 units, every other body in the solar system is the 
appropriate scaled mass comparatively.
Intitial positions are all the same, along the x-axis, scaled relative
to the Earth which begins at 200 units (1 AU). All others begin at their perihelions
such that the velocity vector may be perpendicular to the position.
All initial velocities are chosen such that orbital eccentricities are comfortably
close to the real life values.

Controls:
F - Free/Focus Mode is toggled
V - Top down / Planar view is toggled
R - Camera reset (depending on view mode) is activated
P - Skybox is toggled

Free Mode:
WASD - Movement
Space - Vertical Up movement
Control - Vertical Down movement
Shift - Increase speed in any direction
Left Mouse Hold - Moves the camera in any direction

Focus Mode:
W - Zoom in
S - Zoom out
A/D - Change focus body
Left Mouse Hold - Revolve around focus body

Body Manipulation - Bodies are added in the main file and must be added to the 
array of meshes (and the number of bodies constant must be appropriately set).
Textures must be a perfect box, power of 2 resolution. The radius and mass of the object
and initial positions are also labeled in the main file.
Non-Solar meshes should be shaded with the default shader as they are in the main file.
The render file has a function "Move" which assigns velocities, rotational speed,
and orbital evolution speed (dt). Higher dt results in slower evolution. Lower dt
results in faster evolution.

Methods:
https://learnopengl.com/
Planetary Textures -
https://www.solarsystemscope.com/textures/
Starry SkyBox-
https://svs.gsfc.nasa.gov/4851/
Moon Texture-
https://svs.gsfc.nasa.gov/cgi-bin/details.cgi?aid=4720
Real Modeling Description - 
https://ilrs.gsfc.nasa.gov/docs/2014/196C.pdf
2-body Einstein approximation Method-
Introducing Einstein's Relativity - Ray D'Inverno

Future:
To actually do simulations, SPICE will need to be integrated into the system for accurate
planetary locations. Then, realistic gravitational equations including ODE solvers will
need to be implemented to predict the future orbit of spacecraft that navigate this solar
system. Without the ability to implement spacecraft, which require realistic gravitational
equations and methods for solving them, this program is a glorified demo, and simulates 
exactly nothing.
