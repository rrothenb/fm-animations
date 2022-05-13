import colorsys
from numpy import sin, pi, power, zeros, uint8, int32, float32


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
    x = xIndex/res*2*pi
    y = yIndex/res*2*pi
    z = zIndex/res*2*pi
    return sin(y-z+2*sin(2*y+z+5*sin(3*x)))/2+.5

res = 256

sigmat = zeros((res, res, res, 3))

for z in range(res):
    for y in range(res):
        for x in range(res):
            a = texture(x, y, z, res)
            sigmat[z, y, x] = colorsys.hsv_to_rgb(0, a, power(a, 2))

write_binary_grid3d('textures/sigmat.vol', sigmat)

albedo = zeros((res, res, res, 3))

for z in range(res):
    for y in range(res):
        for x in range(res):
            a = texture(y, z, x, res)
            albedo[x, y, z] = colorsys.hsv_to_rgb(0, power(a, 2), 1-a)

write_binary_grid3d('textures/albedo.vol', albedo)
