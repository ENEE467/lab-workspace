#include "lab8/ur3e_move_interface.hpp"

void UR3eMoveInterface::drawCircleXY(double radius_meters)
{
  if (radius_meters <= 0)
    radius_meters = 0.45;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target

  /// TODO: Set the points on the circle as waypoints and execute a Cartesian plan

  /// TODO: Move the robot back to its home position by setting a named target
}

void UR3eMoveInterface::drawCircleYZ(double radius_meters)
{
  if (radius_meters <= 0)
    radius_meters = 0.45 * 0.5;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target

  /// TODO: Set the points on the circle as waypoints and execute a Cartesian plan

  /// TODO: Move the robot back to its home position by setting a named target
}

void UR3eMoveInterface::drawSquareXY(double side_meters)
{
  if (side_meters <= 0)
    side_meters = (0.45 * 2) * M_SQRT1_2;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target

  /// TODO: Set the corners of the square as waypoints and execute a Cartesian plan

  /// TODO: Move the robot back to its home position by setting a named target
}

void UR3eMoveInterface::drawSquareYZ(double side_meters)
{
  if (side_meters <= 0)
    side_meters = 0.425;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target

  /// TODO: Set the corners of the square as waypoints and execute a Cartesian plan

  /// TODO: Move the robot back to its home position by setting a named target
}
