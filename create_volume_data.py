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


def textureS(xIndex, yIndex, zIndex, res):
    x = xIndex/res*2*pi
    y = yIndex/res*2*pi
    z = zIndex/res*2*pi
    return sin(3*y-2*z+5*sin(2*y+5*z+7*sin(4*x+3*z+2*sin(x+y+z))))/2+.5


def textureV(xIndex, yIndex, zIndex, res):
    x = xIndex/res*2*pi
    y = yIndex/res*2*pi
    z = zIndex/res*2*pi
    return sin(2*x-5*z+6*sin(2*y+3*z+8*sin(6*z-2*y+5*sin(2*x+3*y+4*z))))/2+.5

res = 256

sigmat = zeros((res, res, res, 3))

for z in range(res):
    for y in range(res):
        for x in range(res):
            s = textureS(x, y, z, res)
            v = textureV(x, y, z, res)
            sigmat[z, y, x] = colorsys.hsv_to_rgb(0, s, power(v, 2))

write_binary_grid3d('textures/sigmat.vol', sigmat)

albedo = zeros((res, res, res, 3))

for z in range(res):
    for y in range(res):
        for x in range(res):
            s = textureS(y, z, x, res)
            v = textureV(z, x, y, res)
            albedo[x, y, z] = colorsys.hsv_to_rgb(0, power(s, .5), 1-v)

write_binary_grid3d('textures/albedo.vol', albedo)
