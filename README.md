# Isocline Curve

Find a direction V with a shortest 0° isocline curve

# Approach

Isoline curve is constructed given a reference vector V and an angle A such that
the angle between the normal to surface and V is (pi/2 - A) and we integrate its
length.

In our case, the angle A = 0 and we are looking for a curve with normals
perpendicular to a given V.

Assuming a closed differentiable surface in mesh representation, get all the faces
that have a normal close enough to perpendicular to the given vector and try to
connect them in a non-trivial loop. The shortest of the boundaries of that loop
approximates the isocline for that direction.

# Requirements

Tested with Python 3.10.11 and Docker version 24.0.2

If you do not have pymesh locally just:

run -it --rm -v `pwd`:/src pymesh/pymesh bash

# Run

python /src/find_isocline.py /src/examples/sphere.obj

