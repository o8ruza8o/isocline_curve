# Isocline Curve

Find a direction V with a shortest 0° isocline curve

# Approach

Isoline curve is constructed given a reference vector V and an angle A such that
the angle between the normal to surface and V is (pi/2 - A) and we integrate its
length.

In our case, the angle A = 0 and we are looking for a curve with normals
perpendicular to the given V.

Assuming a closed differentiable surface in mesh representation, get all the faces
that have a normal close enough to perpendicular to the given vector and try to
connect them in a non-trivial loop. The shortest of the boundaries of that loop
approximates the isocline for that direction.

# Requirements

Tested with Python 3.10.11 and openmesh 1.24.3
```
pip install openmesh
```

# Run an example
```
python find_isocline.py examples/bob_isotropic.obj 
```

![Bob isocline](examples/bob00.png)

