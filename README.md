# A Very Small Cat, Having a Day

A 20-second computer-graphics animation of a boxy orange cat: she crouches, walks, pounces, grooms one paw, and then sits by her food bowl while fire, water, and snow drift around her.

Made for **COMP 4411 Computer Graphics**, Fall 2026, at **The Hong Kong University of Science and Technology**.

Watch the finished shot: [`cat_animation.mp4`](cat_animation.mp4)

## What she does

The cat is a hierarchical model carried over from Project 2. Joints, the tail, and the spine are driven by animation curves, so the motion eases in and out instead of popping.

- She sinks into a crouch, then steps into a diagonal walk.
- She gathers herself and leaps.
- She brings one paw up to groom, then settles into a sit.
- The tail follows a little late. The head looks where she is going.

Fire leaves her mouth and water leaves a front paw. Snow falls over the yard. A bit of cloth and fur rides on her back, and a small flock of boids circles overhead.

## Curves and particles

The saved script, `animator.ani`, uses the curve types from the assignment:

| Curve | What it moves |
| --- | --- |
| Bézier | Height, stretch, sit, pounce |
| B-spline | Side-to-side sway and wind |
| Catmull-Rom | Head, tail, and animation speed |
| C2 interpolating | Forward position and the light |
| Linear | Walk, groom, and how many particles appear |

Particles are emitted from the head and the paw, not from the root of the scene. They are integrated with Euler, baked while the shot plays, and can be switched to RK4 from the slider.

## Run it

On macOS, with [FLTK](https://www.fltk.org/) installed:

```bash
brew install fltk
make animator
./animator
```

If `animator.ani` is in this folder, the program loads it and plays. The camera orbit is in `animator.ani.cam`.

The Windows build is still `Animator.sln`.
