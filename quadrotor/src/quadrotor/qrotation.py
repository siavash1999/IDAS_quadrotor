def conjugate(q):
	x, y, z, w = q
	return (-x, -y, -z, w)

def mult(q1, q2):
	x1, y1, z1, w1 = q1
	x2, y2, z2, w2 = q2
	w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
	x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
	y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
	z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
	return x, y, z, w

def qv_mult(q1, v1):
	q2 = v1 + [0]
	return mult(mult(conjugate(q1), q2), q1)[:3]
