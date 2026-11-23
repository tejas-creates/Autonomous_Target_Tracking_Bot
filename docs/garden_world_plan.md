# Garden world plan

Goal: replace the flat grey slab with a backyard garden that looks real (good for GitHub),
without breaking the camera, LiDAR or NMPC.

Status: **plan only, nothing built yet.**

---

## 1. Scale

| Thing | Size | Note |
|---|---|---|
| AGV / Target_AGV | ~0.20 × 0.20 m, ~0.2 m tall | toy-rover size |
| Driving area | 12 × 8 m | already a real backyard size |
| Obstacles now | cylinders r = 0.10 m, h = 0.30 m | already real flower-pot size |
| LiDAR scan plane | ~0.22 m above ground | everything crossing this height is an obstacle |

**Decision: build the garden at real 1:1 scale** and treat the AGV as a small garden rover.
The arena and obstacles already have real garden sizes, so only the *choice* of objects
needs care: small things (pots, stones, edging) near the robot, big things (trees, benches,
lamp posts) as background outside the driving area.

(Alternative: a miniature 1:5 garden — every model must then be scaled down, more work,
and scanned Fuel models look wrong when shrunk. Not recommended.)

---

## 2. Rules that must not break

1. **LiDAR sees visuals.** Anything between ~0.15–0.30 m height inside the area becomes an obstacle.
   → Decoration inside the area is either flat on the ground (texture, ≤ 1 cm) or a real obstacle.
2. **No hidden low obstacles.** Something lower than the scan plane (e.g. a 10 cm flower bed)
   is invisible to the LiDAR but the AGV still collides with it. → Keep those outside the area.
3. **Keep obstacle names and shape.** `dynamic_obstacle_controller` moves `Moving_Cylinder_01..11`
   by name at z = 0.15. New models keep these names, stay round (r ≈ 0.10 m) at scan height,
   and must reach above 0.22 m.
4. **Checkerboard must stay detectable.** Re-test detection after every lighting change;
   no strong specular light on the target's rear panel.
5. **Performance.** Camera is 1920×1080 @ 50 Hz. Plan: 1280×720 @ 15 Hz (board still ~60 px wide at 0.6 m),
   shadows only from the sun, low-poly background models.

---

## 3. Layout

```
   oak / pine trees, lamp post, bench          (background, outside the hedge)
 ┌──────────────────────── hedge 0.5 m ──────────────────────────┐
 │  grass                                                         │
 │     ░░░░ gravel path (follows trajectory.csv) ░░░░            │
 │   (pot)        ░░░░             (pot)    ⊙ rover              │
 │        ⊙ rover       ░░░░  (pot)                    ░░░░       │
 │  (stump)    ░░░░  stepping stones   ░░░░    (pot)              │
 │                      ⊙ ball                                    │
 └────────────────────────────────────────────────────────────────┘
   (pot) = static obstacle     ⊙ = moving obstacle     ░ = flat, no collision
```

- **Ground:** grass texture on the existing 12 × 8 m box.
- **Path:** thin (1 cm) gravel / stepping-stone visuals along the target's waypoints,
  no collision → looks like the target drives on a garden path; LiDAR ignores it.
- **Boundary:** hedge (or low brick wall) 0.5 m tall around the area. This is the first real
  wall in the world (the old readme mentions an obround boundary, but the current world has none);
  the LiDAR package already turns walls into chains of circles.
- **Background:** trees, a bench and a lamp post *outside* the hedge, only for the camera/GUI view.

---

## 4. Obstacles

| Now | Garden version | How |
|---|---|---|
| 7 yellow static cylinders | terracotta flower pots with a plant, tree stumps | scanned pot mesh (Fuel) or own model; collision stays a cylinder r 0.10 h 0.30 |
| 11 red moving cylinders | small round garden robots (mower-like dome, bumper ring, LED) | built from primitives with PBR materials → exactly round, no mesh |
| — (option) | 1–2 rolling footballs | needs per-obstacle height in `dynamic_obstacle_controller` (now fixed z = 0.15) |

Plants on top of pots may be wider than the pot only **above ~0.30 m** (above the AGV and the scan plane).

---

## 5. Assets (Fuel, licences checked)

| Use | Fuel model | Licence |
|---|---|---|
| Background trees | `OpenRobotics/Oak tree`, `OpenRobotics/Pine Tree` | CC0 |
| Grass texture | `OpenRobotics/Grass Plane` | CC-BY 4.0 |
| Flower pots | `GoogleResearch/Cole_Hardware_Flower_Pot_1025`, `Ecoforms_Plant_Pot_GP9_SAND`, … | CC-BY 4.0 |
| Bench | `OpenRobotics/FoodCourtBenchShort` | CC0 |
| Lamp post | `OpenRobotics/Lamp Post` | CC0 |
| Fence (option to hedge) | `athackst/fence` | CC-BY 4.0 |
| Hedge, path, stones, robots | own simple models + CC0 textures (e.g. ambientCG) | CC0 |

- Download once and **store in `simulation/models/`** (works offline, fixed versions).
- CC-BY assets → credits section in the README.
- Check each model's real size before use (scanned pots are real-world size).

---

## 6. Look

- **Sky:** `<sky>` with clouds (ogre2), daylight background instead of purple-grey.
- **Sun:** white-warm directional light from above at an angle (now orange, almost horizontal),
  shadows on; weak ambient fill so shadows are not black.
- **Materials:** PBR (roughness/metalness) for pots, robots, AGVs instead of flat colours.
- **GUI (`gui.config`):** start camera at a nice 3/4 view of the garden, grid hidden,
  image panel with the AGV camera, optional "follow AGV" camera.
- **RViz (`rviz/demo.rviz`):** camera image with detection overlay, `/obstacles/markers`, `/nmpc/path`.

---

## 7. Files

```
simulation/
  worlds/garden.sdf            new world (old simulation_world.sdf stays as fallback)
  models/                      hedge, path, pot, garden_robot, + downloaded Fuel models
  gui/garden.gui.config        GUI layout
  launch/world.launch.py       new argument  world:=garden | plain
ros_ws/rviz/demo.rviz
docs/media/                    screenshots + GIF for the README
```

Small code changes: launch argument for the world, camera resolution in the URDF,
optional obstacle height in `dynamic_obstacle_controller`. Nothing in `lidar` / `nmpc`.

---

## 8. Steps

1. Ground, sky, sun, hedge → check LiDAR sees the hedge as wall circles.
2. Replace static cylinders by pots/stumps → check LiDAR radius ≈ 0.10 m.
3. Replace moving cylinders by garden robots → check moving/static classification.
4. Gravel path along `trajectory.csv`, background trees/bench/lamp.
5. Camera 1280×720 @ 15 Hz → check checkerboard detection and real-time factor.
6. GUI + RViz layouts, full following run, record GIF, README credits.

## 9. Test checklist

- [ ] Checkerboard detected at start and during the run
- [ ] `/obstacles`: pots static, robots moving, hedge = wall circles, no false obstacles from decoration
- [ ] NMPC follows the full trajectory, no contact
- [ ] Real-time factor ≥ 0.8 on this laptop
- [ ] Fresh clone works offline (all models local)

## 10. Open decisions

- Hedge or fence/brick wall as boundary?
- Moving obstacles: garden robots only, or also rolling balls?
- Keep 1920×1080 camera if performance allows?
