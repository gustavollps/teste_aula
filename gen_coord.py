import random

matricula = 2018102432

random.seed(matricula)
x = 0
y = 0

while abs(x) < 0.5 or abs(y) < 0.5:
    x = random.random() * 6 - 3
    y = random.random() * 6 - 3

print(x, y)
