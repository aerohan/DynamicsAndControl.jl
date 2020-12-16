 # DynamicsAndControl.jl

 DynamicsAndControl.jl is an experimental Julia library for closed loop
 simulation of complicated systems - vehicles, robots, or any system
 characterized by continuous nonlinear dynamics and discrete control updates.

 ### Key features
 * Quickly and easily build complicated simulations (complex dynamics,
 discrete controllers running at various rates, sensor and actuator dynamics,
 noise and disturbances, etc.)
 * Simulation interface is at the "type" level - define struct types
 (composite data structures) that can be integrated and updated directly,
 without the need for manually indexing or serializing for numerical
 integration
 * Easily log and plot data from anywhere in the simulation - inside the
 dynamics, the controller, or any other component
 * High performance - core integration and update loop is type stable and
 zero-allocating, and minimal runtime overhead for working with "struct"
 abstractions

 A tutorial/example can be found [here](https://github.com/aerohan/DynamicsAndControl.jl/blob/main/examples/rocket_landing/rocket_landing.ipynb).

 WARNING: currently under active development
