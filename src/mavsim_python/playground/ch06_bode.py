from control import tf, bode_plot
import matplotlib.pyplot as plt


def secondOrder():
    zeta = 0.707  # damping ratio
    wn = 100
    K = 1
    num = [wn ** 2]
    den = [1, 2 * zeta * wn, wn ** 2]

    system = tf(num, den)

    mag, phase, w = bode_plot(system)


def derivative(w):
    num = [1, 0]
    den = [1]

    system = tf(num, den)
    mag, phase, w = bode_plot(system, omega=w)

    return mag, phase, w


def derivative2():
    tau = 0.1

    num = [tau, 0]
    den = [tau, 1]

    system = tf(num, den)
    mag, phase, w = bode_plot(system)

    return mag, phase, w


if __name__ == "__main__":
    # secondOrder()
    _, _, w = derivative2()
    derivative(w)

    plt.show()
