# Isocline Curve

Find a direction a with a shortest 0° isocline curve

# Procedure

Isoline curve is constructed given a reference vector V and an angle A such that
the angle between the normal to surface and V is (pi/2 - A) and we integrate its
length.

In our case, the angle A = 0 so we are looking for normals perpenidicular to a
given V.

Given a closed differentiable surface in mesh representation, get all the faces
that have a normal close enough to perpendicular to the given vector and try to
connect them in a non-trivial loop. The shortest of the boundaries of that loop
approximates the isocline for that direction.

# Requirements

tested with python 3.11 but older version should be just fine.

pip install pymesh

