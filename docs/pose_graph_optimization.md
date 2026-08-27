# Pose-graph optimization

**Pose-graph optimization (PGO)** is probably one of the easiest SLAM concepts to understand once you have the right mental picture.

The key idea is:

> **Pose-graph optimization adjusts all robot poses so that the relative-motion and loop-closure constraints are as consistent as possible.**

Think of it as **"fixing the robot's entire trajectory using a network of geometric constraints."**

---

## 1. First: what is a "pose"?

A robot's **pose** is its position + orientation.

For a 2D robot:

$${x_i = [x,\ y,\ \theta]}$$

For a 3D robot:

$${T_i \in SE(3)}$$

which contains:

* 3D position
* 3D orientation

Imagine the robot traveling:

```text
t₀       t₁       t₂       t₃       t₄

🚗───────🚗───────🚗───────🚗───────🚗
```

Each robot location is a **node**:

```text
x₀ ─── x₁ ─── x₂ ─── x₃ ─── x₄
```

That's the **pose graph**.

---

## 2. Where do the edges come from?

Suppose the robot moves from `x₀` to `x₁`.

From odometry or visual odometry, we estimate:

> "The robot moved approximately 1 meter forward."

That's a constraint between the two poses:

```text
x₀ ───────── x₁
      Δ₀₁
```

Similarly:

```text
x₀ ──Δ₀₁── x₁ ──Δ₁₂── x₂ ──Δ₂₃── x₃
```

Each edge says:

> **"The relative transformation between these two poses should approximately equal this measurement."**

---

## 3. Why do we need optimization?

Because measurements are noisy.

Suppose the robot actually walks in a square:

```text
      x₃ ───── x₄
      │         │
      │         │
      x₂         x₅
      │         │
      │         │
      x₁ ───── x₀
```

But odometry has small errors.

The robot might estimate:

```text
x₀ ───── x₁
          \
           x₂
            \
             x₃
               \
                x₄
```

After enough motion, small errors accumulate.

This is called **drift**.

---

## 4. The really important event: loop closure

Now suppose the robot eventually recognizes:

> "Hey! I've been here before."

For example, it recognizes the same visual landmark/place corresponding to `x₀`.

We obtain a loop-closure constraint:

```text
x₀ ───── x₁ ───── x₂ ───── x₃ ───── x₄
│                                     │
└──────────── loop closure ───────────┘
```

This is extremely valuable.

It says:

> **"According to this observation, x₄ should be near x₀ with approximately this relative orientation."**

But our accumulated odometry says otherwise.

Now we have a conflict.

---

## 5. Pose-graph optimization resolves the conflict

We have:

### Odometry says:

```text
x₀ → x₁ → x₂ → x₃ → x₄
```

### Loop closure says:

```text
x₄ → x₀
```

The measurements aren't perfectly consistent because they're noisy.

So PGO asks:

> **"Can I slightly move all these poses so that all constraints are satisfied as well as possible?"**

This is the crucial intuition.

It doesn't necessarily say:

> "x₄ is wrong."

Instead, it says:

> "Maybe x₁, x₂, x₃ and x₄ are all slightly wrong. Let's distribute the error."

---

## 6. Imagine stretching a rubber band

This is my favorite analogy.

Imagine every pose is a bead:

```text
●────●────●────●────●
```

And every edge is a **rubber band** telling neighboring poses:

> "You should be approximately this far apart and oriented this way."

Then loop closure adds another rubber band:

```text
●────●────●────●────●
│                   │
└───────────────────┘
```

But the rubber bands are pulling in slightly conflicting directions.

If you release the system:

```text
      ●────●
     /      \
    ●        ●
     \      /
      ●────●
```

the beads settle into a configuration that best satisfies all the rubber bands.

That's essentially what optimization is doing.

---

## 7. The mathematics is actually quite intuitive

For every edge, we have:

$${z_{ij}}$$

which is the **measured relative transformation** between poses $i$ and $j$.

Given our current estimates $T_i$ and $T_j$, we can calculate what relative transformation they imply:

$${T_i^{-1}T_j}$$

Then compare:

$${\text{error}_{ij} = z_{ij}^{-1}(T_i^{-1}T_j)}$$

Conceptually:

```text
measured relationship
        ↓
      zᵢⱼ

estimated relationship
        ↓
   Tᵢ⁻¹ Tⱼ

        ↓
     compare

        ↓
      error
```

Then PGO minimizes the total error over every edge in the graph (odometry edges plus loop-closure edges):

$${\boxed{\min_{T_0,\ldots,T_n}\sum_{(i,j) \in \mathcal{E}}\|e_{ij}\|^2}}$$

So in plain English:

> **Find the poses that make all the measured relative transformations agree as much as possible.**

---

## 8. Why is this different from Bundle Adjustment?

This is an extremely important distinction.

### Bundle Adjustment

Optimizes:

```text
Camera poses
     +
3D landmarks
```

using:

```text
image observations
      ↓
reprojection error
```

Conceptually:

```text
        📷 T₀
       /  \
      /    \
    P₁     P₂
     \      /
      \    /
       📷 T₁
```

---

### Pose-graph optimization

Usually optimizes:

```text
Robot poses
     +
relative-pose constraints
```

without explicitly optimizing the 3D landmarks.

```text
T₀ ───── T₁ ───── T₂ ───── T₃
 \                         /
  └────── loop closure ───┘
```

So:

> **BA asks:**
> "Do my cameras and 3D points explain the images?"

> **PGO asks:**
> "Do my robot poses form a trajectory consistent with all the relative-pose measurements?"

---

## 9. Another useful analogy: GPS navigation

Imagine you are reconstructing someone's journey.

You have:

* odometry
* GPS
* landmarks
* loop closures

Your odometry says:

```text
Home → A → B → C → D
```

But accumulated error makes `D` appear 20 m away from Home.

Then GPS tells you:

> "D is actually very close to Home."

Instead of moving only `D`, you could distribute the correction:

```text
Before:

Home ─ A ─ B ─ C ───────── D
                         20m error


After:

Home ─ A ─ B ─ C ─────── D
       ↘   ↘   ↘   ↘
        small corrections
```

The trajectory becomes globally consistent.

That's essentially what PGO does.

---

## 10. Why loop closure is so powerful

Without loop closure:

```text
x₀ ─ x₁ ─ x₂ ─ x₃ ─ x₄ ─ x₅ ─ x₆
```

The graph is basically a chain.

There's not much opportunity to correct accumulated drift.

With loop closure:

```text
        ┌────────────────────┐
        ↓                    │
x₀ ─ x₁ ─ x₂ ─ x₃ ─ x₄ ─ x₅
```

we suddenly have a **cycle**.

That cycle provides a powerful consistency check.

You can think of it as:

> **"If I follow the measurements around this loop, I should eventually come back to where I started."**

If I don't, there is accumulated error.

Optimization distributes that error.

---

## 11. What happens in a real SLAM system?

A typical pipeline looks roughly like:

```text
Camera / LiDAR / IMU
        │
        ▼
 Front-end estimation
        │
        ▼
Relative pose
        │
        ▼
   New pose xᵢ
        │
        ▼
   Pose graph
        │
        ├──────────────┐
        │              │
        ▼              ▼
Odometry edge     Loop closure
        │              │
        └──────┬───────┘
               ▼
        Graph optimization
               │
               ▼
     Corrected trajectory
```

The **front-end** says:

> "I think I moved like this."

The **back-end** says:

> "Let's see whether all those estimates make sense together."

This front-end/back-end separation is very important in SLAM.

---

## 12. One subtle point: PGO doesn't magically know the correct trajectory

Suppose you have:

```text
x₀ ───── x₁ ───── x₂
```

and noisy measurements.

Optimization isn't discovering some objectively "true" trajectory.

It's finding:

> **the trajectory that best satisfies the available constraints according to the chosen error model and weights.**

For example, if one measurement is considered highly reliable:

$${w_1 = 100}$$

and another is noisy:

$${w_2 = 1}$$

the optimizer will care much more about satisfying the first constraint.

So the more complete objective is something like:

$${\min_X \sum_{(i,j) \in \mathcal{E}} e_{ij}^T \Omega_{ij} e_{ij}}$$

where $\Omega_{ij}$ is related to the **information/covariance** of the measurement.

This is why sensor uncertainty matters.

---

## 13. The most important intuition

You can connect the three concepts we've discussed like this:

```text
                SLAM
                 │
        ┌────────┴────────┐
        │                 │
    Filtering         Optimization
        │                 │
        │          ┌──────┴──────┐
        │          │             │
       EKF      Pose Graph       BA
                  │              │
                  │              │
             poses only     poses + landmarks
                  │              │
                  ▼              ▼
             relative       reprojection
              pose errors       errors
```

And the three questions become:

### Filtering

> **"Given everything I've seen so far, where am I now?"**

### Pose-graph optimization

> **"Given all these relative-pose constraints, what trajectory is most consistent?"**

### Bundle adjustment

> **"Given all these images, what camera trajectory and 3D structure best explain the observations?"**

---

## The one-sentence mental model

If you remember only one thing:

> **Pose-graph optimization is like taking a trajectory made of slightly inaccurate pieces, connecting those pieces with constraints—including loop closures—and then moving the poses around until the entire graph becomes as geometrically consistent as possible.**

And there's a particularly important connection to your SLAM research: **PGO is essentially a sparse nonlinear least-squares problem over poses on $SE(2)$ or $SE(3)$**. Once you understand that, the next natural step is understanding **why we need Lie groups / Lie algebra and how Gauss–Newton or Levenberg–Marquardt actually moves the poses during optimization**.
