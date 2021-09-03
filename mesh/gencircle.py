import math

vertices = []
faces = []

level = 100
radius = 2

vertices.append((0, 0, 0))

for i in range(level):
    angle = math.pi * 2.0 / level * i

    vertices.append((
        math.cos(angle) * radius,
        math.sin(angle) * radius,
        0
    ))

for i in range(level):
    faces.append((
        1,
        i + 2,
        i + 3 if i < level - 1 else 2
    ))

lines = []

for v in vertices:
    lines.append(f'v {v[0]} {v[1]} {v[2]}')

for f in faces:
    lines.append(f'f {f[0]} {f[1]} {f[2]}')

with open('Circle.obj', 'w') as p:
    p.write('\n'.join(lines))

print('Done!')
