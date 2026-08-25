
# What are the difference between the standard Kalman Filter, Extended Kalman Filter, and Invariant Extended Kalman Filter? 

The most intuitive way to understand them is to start with one idea:

> **A Kalman Filter is basically a smart way of combining “what I predicted” with “what I measured.”**

The three filters differ mainly in **what kind of system they assume** and **how they deal with nonlinear motion**.

---

## 1. Standard Kalman Filter: “Everything is nicely linear”

Imagine you're tracking a car.

You have:

* a previous estimate: `car is at x = 10 m, moving at 5 m/s`
* a motion model: `after 1 second, it should be around x = 15 m`
* a sensor measurement: `GPS says x = 14 m`

The Kalman Filter asks:

> **How much should I trust my prediction versus my measurement?**

If GPS is noisy, you might say:

> Prediction: 70%
> GPS: 30%

So the result might be around `14.7 m`.

The important assumption is that the relationship between the state and its evolution is **linear**.

For example:

[
x_{k+1} = Fx_k + w
]

and measurement:

[
z_k = Hx_k + v
]

where:

* (x) = state
* (z) = measurement
* (F) = linear motion model
* (H) = linear measurement model
* (w,v) = noise

### Intuition

Think of the standard KF as:

> **“I have a straight ruler, and the world behaves approximately like a straight line.”**

It's elegant and mathematically clean, but it doesn't work directly for things like rotations, camera poses, or nonlinear robot dynamics.

---

# 2. Extended Kalman Filter: “The world is nonlinear, so I'll approximate it locally”

Now suppose your robot is moving.

Its state might be:

[
x = [x,y,\theta]
]

where (\theta) is its orientation.

The motion might look like:

[
x_{k+1}=x_k+v\cos\theta,\Delta t
]

[
y_{k+1}=y_k+v\sin\theta,\Delta t
]

This is **nonlinear** because of the (\cos\theta) and (\sin\theta).

A standard KF can't handle this directly.

So the EKF says:

> “Okay, the system is nonlinear, but perhaps I can pretend it's linear **around my current estimate**.”

It uses a **Jacobian** to locally approximate the nonlinear function.

Imagine a curved road:

```text
                 actual nonlinear function
                       /
                     /
                   /
                __/
             __/
          __/
       __/

        EKF approximation
       /
      /
     /
```

The EKF essentially says:

> “I don't need to understand the whole curve.
> I just need to approximate the curve around where I currently am.”

Mathematically, if:

[
x_{k+1}=f(x_k,u_k)+w
]

the EKF computes:

[
F_k = \frac{\partial f}{\partial x}
]

That's the Jacobian.

Then it uses this local linear approximation inside the normal Kalman equations.

### Intuition

So:

**KF**

> "My world is linear."

**EKF**

> "My world is nonlinear, but I'll locally pretend it's linear."

This works surprisingly well and is probably one of the most widely used nonlinear filtering approaches.

---

# 3. But here's the problem with EKF

This becomes particularly important in **robotics and SLAM**.

Suppose your robot has a pose:

[
X=(R,p)
]

where:

* (R) = rotation
* (p) = position

Rotations are not ordinary vectors.

For example, if you rotate something by 90° and then another 90°:

[
90^\circ + 90^\circ = 180^\circ
]

but rotations in 3D have more complicated geometry.

They live on a mathematical structure called a **Lie group**, typically:

[
SO(3)
]

for rotations and

[
SE(3)
]

for 3D poses.

And this is where the Invariant EKF becomes interesting.

---

# 4. Invariant EKF: “Let's respect the geometry of the problem”

The key insight is:

> **Don't treat a robot pose like an ordinary vector if it isn't one.**

This sounds subtle, but it is extremely important.

Imagine two robots:

```text
Robot A                    Robot B

     ↑                          ↑
     |                          |
     ●                          ●
```

Suppose both robots observe the **same relative motion**.

If you change the global coordinate system:

```text
Before                         After changing frame

     ↑                               ↗
     ●                               ●
```

the physical situation hasn't changed.

The robot doesn't suddenly behave differently just because **you changed your coordinate system**.

A good estimator should therefore behave consistently under these transformations.

This property is related to **invariance**.

---

# 5. The big difference

Here's perhaps the most useful mental model:

| Filter   | Mental model                                                                                   |
| -------- | ---------------------------------------------------------------------------------------------- |
| **KF**   | "The world is linear."                                                                         |
| **EKF**  | "The world is nonlinear, so I'll linearize it."                                                |
| **IEKF** | "The world is nonlinear, so I'll linearize it in a way that respects its geometry/symmetries." |

The IEKF is therefore **not simply “EKF but more accurate.”**

It's a different way of constructing the error and performing the linearization.

---

# 6. The really important difference: how do you define error?

This is probably the most important concept for understanding IEKF.

Suppose your estimated robot orientation is:

[
\hat R
]

and the true orientation is:

[
R
]

### Ordinary EKF thinking

You might think:

[
R-\hat R
]

But subtraction doesn't naturally make sense for rotations.

So EKF implementations often represent the rotation using something like Euler angles or a local perturbation and then linearize.

---

### IEKF thinking

Instead of asking:

> "What is the difference between these two rotations?"

in an ordinary vector sense, we ask:

> **"What rotation would transform my estimate into the true rotation?"**

For example:

[
R = \hat R \exp(\delta\theta^\wedge)
]

where:

[
\delta\theta
]

is a small rotation error.

This is much more natural geometrically.

You are essentially saying:

> "My estimate is here. What small motion on the rotation manifold would take me to the truth?"

That's a much more natural representation for robot motion.

---

# 7. Why is this useful for SLAM?

This becomes especially relevant to your PhD topic because you're dealing with **SLAM and state estimation for robots with unusual motion**.

Consider a robot state:

[
X =
\begin{bmatrix}
R & p & v & b_g & b_a
\end{bmatrix}
]

where:

* (R): orientation
* (p): position
* (v): velocity
* (b_g): gyroscope bias
* (b_a): accelerometer bias

An ordinary EKF has to repeatedly calculate Jacobians around the current estimate.

But the system has important geometric symmetries.

For example, changing the global reference frame shouldn't fundamentally change the robot's physical behavior.

The IEKF tries to construct the estimation error so that these symmetries are handled naturally.

That can give you much better behavior when the system is highly nonlinear.

---

# 8. A nice analogy

Imagine you're navigating Prague with a map. 🙂

### KF

You say:

> "The streets are straight."

Very easy.

---

### EKF

You realize:

> "Okay, streets curve. I'll approximate the street as a straight line around my current location."

Then you repeatedly update your approximation.

---

### IEKF

You realize:

> "Wait. I'm navigating on a **map**, and the important thing is how my position and orientation transform relative to the map."

So instead of pretending the map is just a giant flat vector space, you explicitly use the geometry of the navigation problem.

---

# 9. Another way to visualize the progression

Think about this:

```text
                 State estimation
                       │
                       ▼
              ┌─────────────────┐
              │ Linear system?  │
              └────────┬────────┘
                       │ Yes
                       ▼
                  Standard KF
                       
                       │ No
                       ▼
              ┌─────────────────┐
              │ Nonlinear       │
              │ system          │
              └────────┬────────┘
                       │
                       ▼
                     EKF
             "Linearize locally"
                       │
                       │
          But does the system have
          important geometric
          symmetries?
                       │
                       ▼
                     IEKF
       "Linearize while respecting
              the geometry"
```

---

# 10. The subtle but important point

One misconception is:

> **KF → EKF → IEKF = three levels of accuracy.**

That's not quite right.

It's better to think:

> **KF → different mathematical assumptions**
> **EKF → generic nonlinear approximation**
> **IEKF → geometry-aware nonlinear approximation**

In fact, an IEKF can sometimes have **better convergence and consistency properties** than a conventional EKF because the linearization is aligned with the system's inherent symmetries.

---

## 11. If you remember only three sentences

I'd remember these:

### KF

> **"I can model my system as linear."**

### EKF

> **"My system is nonlinear, but I'll approximate it as linear around my current estimate."**

### IEKF

> **"My system is nonlinear and has geometric structure, so I'll define my errors and linearization in a way that respects that structure."**

And for your SLAM research, the last distinction is particularly important: **robot pose is not just a vector; it has geometry.** That's one of the main reasons IEKF is so attractive for inertial navigation, visual-inertial estimation, and SLAM.
