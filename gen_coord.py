import random

matricula = 201813232

random.seed(matricula)
x = 0
y = 0

while abs(x) < 0.5 or abs(y) < 0.8:
    x = random.random() * 7 - 3.5
    y = random.random() * 7 - 3.5

print(x, y)
