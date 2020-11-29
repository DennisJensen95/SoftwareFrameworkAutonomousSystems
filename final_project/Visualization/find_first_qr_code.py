import matplotlib.pyplot as plt

robot_coordinate = (-0.3, 0.5)
qr_coordinate = (-1, 2.1)
diff = ((qr_coordinate[0]-robot_coordinate[0]),
        (qr_coordinate[1]-robot_coordinate[1]))

line_x = (robot_coordinate[0], qr_coordinate[0])
line_y = (robot_coordinate[1], qr_coordinate[1])

print(diff)

plt.figure()
plt.plot(robot_coordinate[0], robot_coordinate[1], 'rx')
plt.plot(qr_coordinate[0], qr_coordinate[1], 'bx')
plt.plot(line_x, line_y, "k--")

plt.legend(["Robot position", "QR code position", "Heading and distance"])
plt.ylim(-3, 3)
plt.xlim(-3, 3)
plt.grid()


plt.show()
