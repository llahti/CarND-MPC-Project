#ifndef VEHICLE_H
#define VEHICLE_H

const struct VehicleModel {
  // This value assumes the model presented in the classroom is used.
  //
  // It was obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on a
  // flat terrain.
  //
  // Lf was tuned until the the radius formed by the simulating the model
  // presented in the classroom matched the previous radius.
  //
  // This is the length from front to CoG(Center of Gravity) that has a similar radius.
  const double Lf = 2.67;
  // This is pure guess of Lr value
  const double Lr = 2.67;
} Vehicle;

#endif // VEHICLE_H
