import colorsys
from numpy import sin, pi, power, zeros, uint8, int32, float32, absolute


def write_binary_grid3d(filename, values):
    with open(filename, 'wb') as f:
        f.write(b'V')
        f.write(b'O')
        f.write(b'L')
        f.write(uint8(3).tobytes())  # Version
        f.write(int32(1).tobytes())  # type
        f.write(int32(values.shape[0]).tobytes())  # size
        f.write(int32(values.shape[1]).tobytes())
        f.write(int32(values.shape[2]).tobytes())
        if values.ndim == 3:
            f.write(int32(1).tobytes())  # channels
        else:
            f.write(int32(values.shape[3]).tobytes())  # channels
        f.write(float32(0.0).tobytes())  # bbox
        f.write(float32(0.0).tobytes())
        f.write(float32(0.0).tobytes())
        f.write(float32(1.0).tobytes())
        f.write(float32(1.0).tobytes())
        f.write(float32(1.0).tobytes())
        f.write(values.ravel().astype(float32).tobytes())


def texture(xIndex, yIndex, zIndex, res):
    x = float32(xIndex)/res*2.0-1
    y = float32(yIndex)/res*2.0-1
    z = float32(zIndex)/res*2.0-1
    if absolute(x*x*x+y*y*y+z*z*z-1) < .1:
        return 1
    return 0


res = 256

volume = zeros((res, res, res, 3))

for z in range(res):
    if z%8 == 0:
        print(f"{100*z/res}% done")
    for y in range(res):
        for x in range(res):
            t = texture(x, y, z, res)
            volume[x, y, z] = (t, t, t)

write_binary_grid3d('shape.vol', volume)

