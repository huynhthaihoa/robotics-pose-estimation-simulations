# Bundle Adjustment

**Bundle Adjustment (BA)** sounds intimidating, but the intuition is actually quite simple:

> **Bundle Adjustment = jointly adjusting camera poses and 3D points so that the observed image measurements are explained as accurately as possible.**

The key word is **jointly**.

---

## 1. Start with a simple camera + 3D point

Imagine a camera looking at a 3D point:

```text
        3D point
           ● P
          / 
         /
        /
       📷 Camera
```

The 3D point `P` gets projected onto the camera image:

```text
3D world                 Image

    P ●                    • p
      \                    /
       \                  /
        \                /
         📷 ------------ image plane
```

If we know:

* camera pose
* camera intrinsics
* 3D point position

we can predict where `P` should appear in the image.

Mathematically:

$${p = \pi(TP)}$$

where:

* `P` = 3D point
* `T` = camera pose
* `π` = camera projection function
* `p` = predicted 2D pixel location

---

## 2. But our estimates are imperfect

Suppose the actual image measurement is:

```text
observed point:       ●
predicted point:          ×
```

There is an error:

```text
        ● observed

           ↕ error

              × predicted
```

This is called the **reprojection error**.

For one observation:

$${e = p_{\text{observed}} - p_{\text{predicted}}}$$

BA tries to make this error as small as possible.

---

## 3. Now add multiple cameras

Suppose the camera moves:

```text
Camera 1          Camera 2          Camera 3

  📷                📷                📷
   \                 \                 \
    \                 \                 \
     ● P               ● P               ● P
```

The same physical 3D point is observed from multiple camera positions.

Each camera produces a 2D observation:

```text
Camera 1 → p₁
Camera 2 → p₂
Camera 3 → p₃
```

We want to find:

* Camera 1 pose
* Camera 2 pose
* Camera 3 pose
* 3D position of P

such that **all projections agree with the observations**.

---

## 4. Here's the important part: both cameras AND points are adjusted

Suppose our initial reconstruction is wrong:

```text
Camera poses:

📷₁       📷₂       📷₃

 \         |         /
  \        |        /
   \       |       /
       ● P
```

Maybe the camera poses are slightly wrong.

Maybe the 3D point is slightly wrong.

Maybe **both** are wrong.

BA doesn't say:

> "The cameras are correct; I'll fix the points."

or:

> "The points are correct; I'll fix the cameras."

Instead:

> **"I'll adjust everything together until the entire reconstruction explains the image measurements as well as possible."**

That's bundle adjustment.

---

## 5. Why is it called "bundle" adjustment?

There's a beautiful geometric intuition.

Each image observation defines a **ray** from the camera through the observed pixel:

```text
Camera 1
   📷
    \
     \
      \       ● 3D point
       \     /
        \   /
         \ /
```

Another camera gives another ray:

```text
📷₁ --------\
             \
              ● P
             /
📷₂ --------/
```

Ideally, the rays intersect exactly at the 3D point.

But because of noise and imperfect estimates:

```text
📷₁ --------\
             \       ●
              \
               \

📷₂ -----------\ 
```

They don't intersect perfectly.

You can think of BA as adjusting the **bundle of rays** and camera poses so that everything fits together better.

Hence:

> **Bundle Adjustment.**

---

## 6. A more useful SLAM example

Imagine a robot/camera moving through a room:

```text
t₀       t₁       t₂       t₃
📷       📷       📷       📷
 \        \        \        \
  \        \        \        \
   ● A      ● B      ● C      ● D
    \       |        / 
     \      |       /
        landmarks
```

The camera observes many landmarks:

```text
        ● L1

📷₀             ● L2
    \          /
     \        /
      ● L3
```

Every observation creates a constraint:

```text
camera pose + 3D landmark
              ↓
       predicted pixel
              ↓
       compare with
       observed pixel
```

So the system has potentially **thousands or millions of constraints**.

BA solves:

$${\min_{\{T_i\},\{P_j\}} \sum_{(i,j) \in \mathcal{O}} \left\|z_{ij} - \pi(T_i P_j)\right\|^2}$$

where:

* $T_i$ = pose of camera `i`
* $P_j$ = 3D landmark `j`
* $z_{ij}$ = observed pixel
* $\pi(T_iP_j)$ = predicted pixel
* $\mathcal{O}$ = the set of (camera, landmark) pairs that were actually observed — not every camera sees every landmark, so the sum only runs over real observations, not all $i,j$ combinations

In plain English:

> **Find the camera poses and 3D points that make the predicted image points match the actual image points as closely as possible.**

---

## 7. Why BA is so powerful

Suppose your estimated trajectory looks like:

```text
Initial:

📷──📷──📷──📷──📷
               \
                \
                 ● landmarks
```

But the actual observations suggest that the cameras should be slightly different:

```text
Optimized:

📷
  \
   📷
     \
      📷
        \
         📷
           \
            📷
```

At the same time, the landmarks move too.

So BA might effectively do:

```text
             Before             After

Camera 1       ×                  ●
Camera 2       ×                  ●
Camera 3       ×                  ●
Camera 4       ×                  ●

Landmark A     ×                  ●
Landmark B     ×                  ●
Landmark C     ×                  ●
```

Everything moves together to minimize the total reprojection error.

---

## 8. Connection to SLAM optimization

This is exactly why BA is an **optimization-based SLAM technique**.

Remember our previous discussion:

> Filtering → maintain the current belief.

> Optimization → maintain many states and jointly improve them.

BA is the latter.

You have a giant optimization problem:

```text
        Camera poses
             ↓
     T₀ T₁ T₂ T₃ T₄
      ↘  ↓  ↙ ↘  ↓
       landmarks
      P₀ P₁ P₂ P₃
             ↓
       projection
             ↓
     predicted pixels
             ↓
     compare with data
             ↓
      total error
             ↓
       optimization
             ↓
      better poses +
      better points
```

---

## 9. BA vs Pose Graph Optimization

This distinction is particularly useful in SLAM.

### Pose graph optimization

Usually optimizes:

$${T_0,T_1,\ldots,T_n}$$

using relative pose constraints:

```text
T₀ ───── T₁ ───── T₂ ───── T₃
 \                         /
  └────── loop closure ───┘
```

The landmarks may already have been marginalized or aren't explicitly part of the optimization.

---

### Bundle Adjustment

Optimizes:

$${\boxed{\text{camera poses + 3D landmarks}}}$$

```text
        T₀       T₁       T₂
         \        |        /
          \       |       /
           P₁    P₂    P₃
```

using **image reprojection errors**.

So a useful mental distinction is:

> **Pose graph:** "Make the poses geometrically consistent."
> **Bundle adjustment:** "Make the entire camera + 3D structure explain the images."

---

## 10. Why BA can be computationally expensive

Imagine:

* 1,000 camera poses
* 100,000 landmarks
* millions of image observations

Then you're optimizing a huge number of variables.

But there's a very useful structure:

```text
Camera variables ─── Landmark variables
       ↕                    ↕
       └──── observations ──┘
```

Camera `T₁` only directly interacts with the landmarks it observes.

That produces a **sparse optimization problem**.

This sparsity is one of the fundamental reasons efficient BA algorithms are possible.

---

## 11. The deepest intuition

Here's how I'd recommend thinking about BA:

Imagine you have a pile of photographs and you're trying to reconstruct a miniature 3D world.

You initially make a rough reconstruction:

```text
     camera       camera
       📷           📷
        \           /
         \         /
          ●       ●
             landmarks
```

Then you ask:

> "If this really were the correct 3D world, would these cameras really see these landmarks at exactly these pixels?"

If not, something is wrong.

Maybe:

* camera 1 is slightly misplaced
* camera 2 is rotated incorrectly
* landmark 1 is too far away
* landmark 2 is too high

So you continuously adjust:

```text
camera poses
     +
3D landmarks
     ↓
projection
     ↓
image error
     ↓
optimization
     ↓
repeat
```

until the reconstruction becomes as consistent with the images as possible.

That's **Bundle Adjustment**.

---

## One sentence to remember

> **Bundle Adjustment is the process of jointly refining camera poses and 3D landmarks so that their projections agree as closely as possible with the observed image features.**

And the conceptual hierarchy is:

```text
SLAM
 │
 ├── Filtering
 │     └── EKF-SLAM
 │
 └── Optimization / Smoothing
       │
       ├── Pose Graph Optimization
       │
       └── Bundle Adjustment
              │
              ├── optimize camera poses
              └── optimize 3D landmarks
```

For **visual SLAM**, BA is essentially the workhorse behind the idea of *"make my entire reconstructed 3D world and camera trajectory agree with all the pixels/features I've observed."*
