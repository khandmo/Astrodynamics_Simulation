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
system file holds the names of the kernels I used at the top and all kernels can be found at the link below. If 
you want to use the sim through different time periods, just modify those lines.

https://naif.jpl.nasa.gov/pub/naif/generic_kernels/<br><br><br>

## Horizons Usage

This project can use data from NASA's Horizons System to implant real spacecraft into the simulation space. The
link is at the bottom of this section. The following settings will need to be in place and the following explanation
will prepare the file for input into the program. 

The Ephemeris Type should be "Vector Table"
The Coordinate Center should be Geocentric, code 500.
The Step Time should be 6 hours, no matter the range of dates.
The Vector Table Settings should return the (2) State vector. Uncertainties are not necessary.
Reference Frame should be ICRF, plane is ecliptic (standard obliquity, inertial).
Output Units in km and seconds.

Eliminate the text file above the name of the first trajectory. There should be 2 spaces before the text in each line
of the trajectories. The line directly under the final trajectory should be "\*\*\*". This is immediately followed by the first
line of state data. The line directly under the final line of state data should be "\*\*\*". This should terminate the file.

Input the file into the program by going to System.cpp in the source code and adding<br>
initSat("*satellite name*", "*file*.txt");<br>
underneath the "initPersistSats" function call in the System class constructor.

https://ssd.jpl.nasa.gov/horizons/app.html#/<br><br><br>

## Dependencies

- C SPICE Toolkit<br>
https://naif.jpl.nasa.gov/naif/toolkit.html <br>

- ImGui from ocornut, popular modular GUI library<br>
https://github.com/ocornut/imgui <br>

- Lines from mhalber, to render (geometry shader) lines in OpenGL.<br>
https://github.com/mhalber/Lines/tree/master <br><br><br>


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
C - Decrease speed in any direction<br>
Left Mouse Hold - Moves the camera in any direction<br>

#### Focus Mode:
Scroll wheel - Zoom <br>
A/D - Change focus body<br>
Left Mouse Hold - Revolve around focus body<br>

Celestial Bodies are added in the system.cpp file, follow the scheme of the others to add new ones (within SPICE).
Textures must be a perfect box, power of 2 resolution, if not the default grey. The radius and mass of the object
and initial positions are also labeled in the main file.
Non-Solar meshes should be shaded with the default shader as they are in the main file.<br><br>

### Links / References
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
https://ilrs.gsfc.nasa.gov/docs/2014/196C.pdf<br><br><br>


### Known Problems / FAQ
---

- Using the GUI to go to a maneuver on a sat and letting some uncertain amount of time pass may result
in a spotaneous inTime = false scenario, ending the sat's life and resetting focus to the Sun.<br>

- GUI satellite parameters like apoapsis / periapsis information as well as close approach time and distance
can be wrong. <br>

- Planet and moon orbits can be wrong / have chunks raised or lowered out of the normal plane. All these bugs
are related to the "refined list" which is supposed to account for smoother orbits near the body itself.
This list can be connected to a main and less smooth orbit.<br>

- Small moons look like that because the simulation units are relatively small and accuracy is hard to come by. No
reason it can't be changed. I think it looks like telescopic imagery from far away so I don't mind.<br>


### Future Plans
---

- improving the accuracy of the data sent to the GUI
- making the state update infrastructure more robust through both forward and backward time travel
- energy replenishment for trajectory accuracy
- smarter time step algorithms
- show planet's current position when zoomed out. Fix refined list problems on orbital paths of bodies.
- add initial dry mass, wet, engine specs, calculate delta-V in initialization process of a satellite, and operate missions under those constraints - including exotic propulsion methods like ion and nuclear
- tools for setting up gravity assists, providing transfer windows for mission objectives depending on the amount of delta-V willing to be used / elapsed time constraints 
- next / last orbit preview to see orbital precession
- change colors of trajectories through different spheres of influence
- align rotational axes with true values



