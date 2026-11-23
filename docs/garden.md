The Autonomous Backyard
Garden World Design Brief
Purpose
Create a realistic, attractive, real-scale suburban backyard environment
for an autonomous mobile-robot simulation.

The result should look like a believable residential garden that happens
to be used by small autonomous garden robots. It should not look like a
miniature toy garden, a generic empty plane with props scattered around
it, or a futuristic science-fiction scene.

Prioritise visual realism, coherent landscaping and believable object
placement while keeping the environment suitable for robotics
simulation.
1. Overall scale and composition
Create a rectangular garden approximately 12 m × 8 m, at real-world
1:1 scale.

The robots are small garden rovers, approximately:
- Footprint: 0.20 m × 0.20 m.
- Height: approximately 0.20 m.
The garden should have a clear central operating lawn surrounded by
landscaping. Keep the main lawn open enough for robots to move around
naturally.

Use realistic proportions. Trees, planters, benches, paths and other
objects should look like full-size garden features, not scaled-down
miniature props.

The composition should be attractive from: - An elevated
three-quarter/isometric view showing the whole garden. - A lower
robot-level view from within the garden.

Avoid placing large decorative objects where they obscure the central
operating area.
2. Garden layout
Organise the environment into four visually coherent zones.
A. Central lawn
The central area is a broad, natural green lawn.
- Use a realistic grass material with subtle colour and texture
  variation.
- Keep the physical ground perfectly flat.
- Do not use raised grass blades or dense individual grass geometry.
- Avoid excessive texture repetition.
- The lawn is the primary robot operating area.
The ground should look natural but remain simple and efficient to
render.
B. Winding garden pathway
Add a narrow, gently winding garden path through the lawn.

The path should combine: - Fine gravel. - Irregular flat stepping
stones. - Subtle natural edging. - Slight variation in stone colour and
shape.

The path should feel like a real backyard walking route rather than a
perfectly straight road.

Where possible, arrange its visual route to complement the existing
robot trajectory, without requiring changes to that trajectory.

Important: the gravel and stepping stones are decorative ground
details. Keep them flat and visual-only, with no raised geometry or
collision hazards. They must not intersect the horizontal LiDAR scan
plane approximately 0.22 m above ground.
C. Garden features
Distribute seven static garden obstacles naturally around the operating
lawn.

Use a visually varied combination of: - Terracotta flower pots with
plants. - Concrete or stone planters. - Small tree stumps. - Rustic
garden containers.

They should look like objects that would naturally be found in a
backyard. Avoid arranging them in a regular grid or making them all
identical.

Plants may extend above the obstacle body, but keep their lower geometry
and collision behaviour safe for the robot. Do not introduce low,
physically collidable objects that the LiDAR cannot detect.
D. Garden perimeter and background
Surround the operating lawn with a continuous, neatly maintained green
hedge approximately 0.5 m high.

Place the hedge outside the usable 12 m × 8 m operating area so it does
not reduce the intended driving space.

Beyond the hedge, create a decorative backyard perimeter containing: -
Mature oak trees. - Pine trees. - A wooden garden bench. - A tasteful
outdoor lamp post. - Optional wooden trellis or pergola with climbing
plants. - Optional bird bath or compost station. - Additional flower
beds and background greenery.

These are background features. Keep them outside the robot operating
area.

The hedge should look dense and natural, but it must also have
sufficiently continuous geometry for predictable LiDAR returns. Do not
rely exclusively on sparse leaves or transparent foliage cards to form
the boundary.
3. Static obstacle appearance
The seven static obstacles should look like a varied collection of
garden objects rather than seven identical cylinders.

Suggested visual variations:
1. Large terracotta planter with a leafy plant.
2. Smaller terracotta flower pot.
3. Weathered concrete planter.
4. Rustic tree stump.
5. Stone-textured planter.
6. Dark ceramic garden pot.
7. Another natural-looking stump or planter.
The visual models can be detailed, but their physical collision
footprint must remain equivalent to the existing cylindrical obstacles:
- Radius: approximately 0.10 m.
- Height: approximately 0.30 m.
Use simple cylindrical collision geometry behind the detailed visual
appearance.

Keep plants and decorative parts from introducing unintended collision
or sensor geometry near the robot.
4. Dynamic obstacle appearance
Include eleven small autonomous garden-maintenance robots moving around
the garden.

They should resemble compact robotic lawn mowers or autonomous garden
rovers.

Visual characteristics: - Rounded, low-profile housing. - Protective
bumper ring. - Small LED indicators. - Subtle wheel details. - Realistic
green, charcoal, black and yellow colour variants. - Matte or slightly
rough outdoor-grade materials.

Create approximately three visual variants so the scene does not look
repetitive.

All eleven robots must retain equivalent sensor-relevant and physical
geometry:
- Circular collision radius: approximately 0.10 m.
- Height: approximately 0.30 m.
- Centre height: approximately 0.15 m.
- Geometry must intersect the LiDAR scan plane at approximately 0.22
  m.
The visual housing may differ between variants, but do not allow the
variants to have substantially different collision footprints or LiDAR
cross-sections.

Keep these robots visually simple. Avoid unnecessary articulated parts,
animated wheels or complicated mechanisms.

Do not add rolling footballs or other spherical moving obstacles in this
version.
5. Boundary and sensor behaviour
The horizontal LiDAR scan plane is approximately 0.22 m above the
ground.

Design the environment with this in mind.
- Objects intersecting this height may produce LiDAR returns.
- Ground textures and flat path details should not create unintended
  returns.
- Low decorative objects can be invisible to LiDAR while still
  physically collidable; do not create such hidden hazards inside the
  operating area.
- The hedge must provide sufficiently continuous returns for boundary
  detection.
- Trees and background objects should remain outside the operating
  area.
- Plants, branches and leaves must not unexpectedly cross the scan
  plane near the robots.
Treat visual geometry, collision geometry and LiDAR visibility as
separate concerns.

Do not assume that a visual-only object is automatically invisible to
GPU LiDAR. The rendered geometry and actual sensor behaviour must be
considered.
6. Lighting, sky and atmosphere
Create a natural late-morning or afternoon outdoor appearance.
Sky
- Natural blue sky.
- Subtle clouds.
- Daylight atmosphere.
- Avoid a purple-grey or overly dark background.
Sun
- One warm-neutral directional sunlight source.
- Sensible sun elevation, not nearly horizontal.
- Soft, natural shadows.
- Moderate ambient fill so shaded areas remain visible.
Materials
Use realistic physically based materials where supported:
- Grass: natural green with moderate roughness.
- Gravel: varied grey and beige stones.
- Stepping stones: slightly weathered stone.
- Pots: rough terracotta or ceramic.
- Stumps: natural weathered wood.
- Bench: outdoor wood.
- Garden robots: matte painted housings.
Avoid excessive gloss, metallic reflections and oversaturated colours.
Checkerboard visibility
The target robot's rear checkerboard must remain easy to detect.
- Avoid strong specular lighting on the rear panel.
- Avoid heavy shadows falling across the checkerboard.
- Avoid dramatic exposure changes between sunlit and shaded areas.
- Do not alter the checkerboard pattern or dimensions as part of the
  garden design.
7. Visual quality and performance
The scene should be visually convincing without becoming unnecessarily
expensive to render.

Use: - Efficient low- or medium-poly background trees. - Reasonably
sized local textures. - Simple collision geometry. - Limited transparent
foliage. - Shadows primarily from the sun. - Reusable materials and
model variants.

Avoid: - Dense individual grass blades. - Excessively detailed scanned
meshes. - Huge texture files. - Unnecessary animated objects. - Large
numbers of shadow-casting decorative objects. - Complex geometry where
it does not improve the visible result.

The simulation should aim for a real-time factor of at least 0.8 on
the target laptop.

The camera's existing configuration is 1920 × 1080 at 50 Hz. Do not
assume it should be changed as part of the garden design. Any later
camera optimisation must be tested separately against checkerboard
detection and tracking quality.
8. Desired visual identity
The finished environment should communicate:

"A real suburban backyard used as an outdoor test site for autonomous
garden robots."

The scene should feel: - Natural. - Lived-in. - Calm and uncluttered. -
Realistically landscaped. - Technically credible as a robotics test
environment.

Use a coherent palette of natural greens, earthy browns, terracotta,
muted stone and charcoal.

Avoid placing objects merely to fill empty space. Every visible feature
should contribute to the garden's layout and atmosphere.
9. Final visual acceptance criteria
The garden design is successful when:
- The complete 12 m × 8 m operating area is visually clear.
- The central lawn looks natural and remains physically flat.
- The pathway looks integrated into the landscaping.
- The hedge forms a continuous perimeter outside the operating area.
- The seven static obstacles look like varied garden objects.
- The eleven moving obstacles look like small autonomous garden
  robots.
- Background trees, bench and lamp post make the scene feel like a
  real backyard.
- Object proportions look realistic at 1:1 scale.
- The checkerboard remains clearly visible under the lighting.
- Decorative ground details do not create unintended LiDAR returns or
  collision hazards.
- The hedge and obstacle geometry remain suitable for LiDAR sensing.
- The scene remains efficient enough for real-time robotics
  simulation.
Design priority: preserve the robotics simulation's existing
physical and sensor assumptions while making the world look like a
coherent, realistic backyard.