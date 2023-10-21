import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider


def simple_mult(x, y):
    return x*y


def u_N_Amp(x, k, A, H=1):
    normlalization_var = np.sqrt(4*k/27)
    arr = A * np.sqrt(np.abs(x))*(H-x/k)*np.sign(x)/normlalization_var
    return arr


def u(x, H, k, func):
    arr = H * np.sqrt(np.abs(x))*(1-x/func(H, k))*np.sign(x)
    return arr


init_k = 1
init_H = 1
init_A = 1

p_delta = np.linspace(-0.03, 1, 100)

fig, ax = plt.subplots()
ax.grid(1)
line, = ax.plot(p_delta, u_N_Amp(p_delta, init_k, init_A))
ax.set_xlabel('Angle [deg]')
fig.subplots_adjust(left=0.25, bottom=0.25)

axA = fig.add_axes([0.1, 0.25, 0.0225, 0.63])
A_slider = Slider(
    ax=axA,
    label="Amplitude",
    valmin=0.1,
    valmax=1,
    valinit=init_A,
    orientation="vertical"
)
axK = fig.add_axes([0.25, 0.1, 0.65, 0.03])
K_slider = Slider(
    ax=axK,
    label="Spring Constant",
    valmin=0.1,
    valmax=1,
    valinit=init_k
)


def update(val):
    line.set_ydata(u_N_Amp(p_delta, K_slider.val, A_slider.val
                           ))
    fig.canvas.draw_idle()


K_slider.on_changed(update)
A_slider.on_changed(update)

# resetax = fig.add_axes([0.8, 0.025, 0.1, 0.04])
# button = Button(resetax, 'Reset', hovercolor='0.975')
# def reset(event):
#     H_slider.reset()
#     K_slider.reset()
# button.on_clicked(reset)

plt.show()
