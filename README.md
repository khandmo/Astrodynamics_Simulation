## Astrodynamics Program

Visually represents a replica solar system with nothing but C++, OpenGL and GLFW libraries for rendering.

Celestial body positions, velocities, and spin rates all from the NASA SPICE Toolkit, allowing data for all
bodies to be collected between the times of ~1970 to ~2050. There are no hard bounds on the system software.

Artificial Satellites hold states of position and velocity and uses algorithms to dynamically select timeSteps
depening on that state relative to the main gravitational body. It uses the Sun and 8 planets and any close moons
and sums their gravitational attraction depending on distance and mass, using the 1PN approximation of Einstein's
field equations. The integration scheme is the 4th order Runge Kutta. Units are kilometers unless otherwise 
stated. There is an in-simulation unit scheme used for display and functions to translate back and forth when
needed.<br><br><br>


## SPICE Kernels

This repo does not contain the kernels for the SPICE Toolkit to obtain data for the celestial bodies. The 
system file holds the names of the kernels I used at the top and all kernels can be found a the link below. If 
you want to use the sim through different time periods, just modify those lines.

https://naif.jpl.nasa.gov/pub/naif/generic_kernels/<br><br><br>

## Horizons Usage

This project can use data from NASA's Horizons System to implant real spacecraft into the simulation space. The
link is at the bottom of this section. The following settings will need to be in place and the following explanation
will prepare the file for input into the program. 

The Ephemeris Type should be "Vector Table"
The Coordinate Center should be Geocentric, code 500.
The Step Time should be 6 hours no matter the range of dates.
The Vector Table Settings should return the (2) State vector. Uncertainties are not necessary.
Reference Frame should be ICRF, plane is ecliptic (standard obliquity, inertial).
Output Units in km and seconds.

Eliminate the text file above the name of the first Trajectory. There should be 2 spaces before the text in each line
of the trajectories. The line directly under the final trajectory should be "***". This is immediately followed by the first
line of state data. The line directly under the final line of state data should be "***". This should terminate the file.

https://ssd.jpl.nasa.gov/horizons/app.html#/<br><br><br>


### Controls (use at your own discretion)
---
F - Free/Focus Mode is toggled<br>
V - Top down / Planar view is toggled<br>
R - Camera reset (depending on view mode) is activated<br>
P - Skybox is toggled<br>

#### Free Mode:
WASD - Movement<br>
Space - Vertical Up movement<br>
Control - Vertical Down movement<br>
Shift - Increase speed in any direction<br>
Left Mouse Hold - Moves the camera in any direction<br>

#### Focus Mode:
W - Zoom in<br>
S - Zoom out<br>
A/D - Change focus body<br>
Left Mouse Hold - Revolve around focus body<br>

Body Manipulation - Bodies are added in the main file and must be added to the 
array of meshes (and the number of bodies constant must be appropriately set).
Textures must be a perfect box, power of 2 resolution. The radius and mass of the object
and initial positions are also labeled in the main file.
Non-Solar meshes should be shaded with the default shader as they are in the main file.
The render file has a function "Move" which assigns velocities, rotational speed,
and orbital evolution speed (dt). Higher dt results in slower evolution. Lower dt
results in faster evolution.<br><br>

### Links
---

#### Methods:
https://learnopengl.com/
#### Planetary Textures -
https://www.solarsystemscope.com/textures/
#### Starry SkyBox-
https://svs.gsfc.nasa.gov/4851/
#### Moon Texture-
https://svs.gsfc.nasa.gov/cgi-bin/details.cgi?aid=4720
#### Real Modeling Description - 
https://ilrs.gsfc.nasa.gov/docs/2014/196C.pdf<br><br><br><br>


### Known Problems
---

Using the GUI to go to a maneuver on a sat and letting some uncertain amount of time pass may result
in a spotaneous inTime = false scenario, ending the sat's life and resetting focus to the Sun.<br><br>

GUI satellite parameters like apoapsis / periapsis information as well as close approach time and distance
can be wrong. <br><br>

Planet and moon orbits can be wrong / have chunks raised or lowered out of the normal plane. All these bugs
are related to the "refined list" which is supposed to account for smoother orbits near the body itself.
This list can be connected to a main and less smooth orbit.<br><br>



